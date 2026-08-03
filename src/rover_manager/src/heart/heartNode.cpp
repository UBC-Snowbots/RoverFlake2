#include "heart/heartNode.h"
#include "rover_utils/include/time_utils.h"
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <cstring>
using namespace ConsoleFormat;

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeartNode>());
    rclcpp::shutdown();
    return 0;
}

HeartNode::HeartNode() : Node("broken_heart",
    rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
{
    std::string my_name = this->get_name();
    if(my_name == "broken_heart"){
        RCLCPP_ERROR(this->get_logger(), "Launch me with name=heart_<host> (see rover_manager launch files).");
        rclcpp::shutdown();
        return;
    }
    my_host_id = my_name;
    my_host_id.erase(0, 6);  // "heart_onboard_nuc" -> "onboard_nuc"

    std::string command_topic = "/heart/command", status_topic = "/heart/status";
    double heartbeat_rate = 1.0;
    this->get_parameter_or("command_topic", command_topic, command_topic);
    this->get_parameter_or("status_topic", status_topic, status_topic);
    this->get_parameter_or("heartbeat_rate", heartbeat_rate, heartbeat_rate);

    std::map<std::string, rclcpp::Parameter> params;
    this->get_parameters("subsystems", params);
    for (const auto& [key, param] : params) {
        RCLCPP_INFO(this->get_logger(), "Subsystem %s%s%s = %s%s%s",
                    green(), key.c_str(), reset(), bright_blue(), param.value_to_string().c_str(), reset());
        SubSystem s;
        s.name = key;
        s.exec_command = param.value_to_string();
        subsystems[key] = s;
    }
    if(subsystems.empty()){
        RCLCPP_WARN(this->get_logger(), "No subsystems defined — did the params file load? Exiting.");
        rclcpp::shutdown();
        return;
    }

    command_sub = this->create_subscription<rover_msgs::msg::SubsystemCommand>(
        command_topic, 10, std::bind(&HeartNode::commandCallback, this, std::placeholders::_1));
    status_pub = this->create_publisher<rover_msgs::msg::HeartStatus>(status_topic, 10);
    heartbeat_timer = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / heartbeat_rate), std::bind(&HeartNode::publishStatus, this));
    supervise_timer = this->create_wall_timer(
        std::chrono::milliseconds(250), std::bind(&HeartNode::superviseTick, this));

    RCLCPP_INFO(this->get_logger(), "I am %s: %zu subsystems, cmd %s, status %s",
                this->get_name(), subsystems.size(), command_topic.c_str(), status_topic.c_str());
}

void HeartNode::startSubsystem(SubSystem& s){
    if(s.state == State::RUNNING || s.state == State::STOPPING || s.state == State::STUCK){
        RCLCPP_WARN(this->get_logger(), "%s is %s — not starting", s.name.c_str(),
                    s.state == State::RUNNING ? "running" : s.state == State::STOPPING ? "stopping" : "stuck");
        return;
    }
    pid_t pid = fork();
    if (pid == 0) {
        if (setsid() < 0) { perror("setsid failed"); _exit(EXIT_FAILURE); }
        execlp("/bin/sh", "/bin/sh", "-c", s.exec_command.c_str(), (char *)NULL);
        perror("execlp failed");
        _exit(127);
    } else if (pid < 0) {
        RCLCPP_ERROR(this->get_logger(), "fork failed for %s: %s", s.name.c_str(), strerror(errno));
        return;
    }
    s.pid = pid;
    s.state = State::RUNNING;
    s.exit_code = 0;
    s.stop_stage = 0;
    s.started_at = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Started %s (pid %d)", s.name.c_str(), pid);
    publishStatus();
}

void HeartNode::stopSubsystem(SubSystem& s){
    if(s.state != State::RUNNING){
        RCLCPP_WARN(this->get_logger(), "%s is not running", s.name.c_str());
        return;
    }
    signalGroup(s.pid, SIGINT);
    s.state = State::STOPPING;
    s.stop_stage = 1;
    s.escalate_at = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    publishStatus();
}

void HeartNode::signalGroup(pid_t pgid, int sig){
    if(pgid <= 0 || pgid == getpgrp()){
        RCLCPP_ERROR(this->get_logger(), "Refusing to signal pgid %d (own group?)", pgid);
        return;
    }
    if (killpg(pgid, sig) < 0)
        RCLCPP_ERROR(this->get_logger(), "killpg(%d, %d): %s", pgid, sig, strerror(errno));
}

void HeartNode::superviseTick(){
    auto now = std::chrono::steady_clock::now();
    bool changed = false;
    for (auto& [name, s] : subsystems) {
        if (s.pid <= 0) continue;
        int wstatus = 0;
        pid_t r = waitpid(s.pid, &wstatus, WNOHANG);
        if (r == s.pid) {
            pid_t pgid = s.pid;  // capture before clearing — needed for the sweep below
            s.exit_code = WIFEXITED(wstatus) ? WEXITSTATUS(wstatus)
                        : WIFSIGNALED(wstatus) ? 128 + WTERMSIG(wstatus) : -1;
            bool was_stopping = (s.state == State::STOPPING || s.state == State::STUCK);
            // final sweep: catch trap-immune grandchildren the tracked pid alone can't signal for;
            // errors (e.g. ESRCH, group already empty) are expected and ignored
            if (s.stop_stage >= 1) killpg(pgid, SIGKILL);
            s.pid = -1;
            s.stop_stage = 0;
            s.state = was_stopping ? State::STOPPED : State::CRASHED;
            if (s.state == State::CRASHED)
                RCLCPP_ERROR(this->get_logger(), "%s CRASHED (exit %d)", s.name.c_str(), s.exit_code);
            else
                RCLCPP_INFO(this->get_logger(), "%s stopped (exit %d)", s.name.c_str(), s.exit_code);
            changed = true;
            if (s.pending_restart) {
                s.pending_restart = false;
                startSubsystem(s);
            }
        } else if (r < 0) {
            RCLCPP_WARN(this->get_logger(), "waitpid(%d) for %s: %s — treating as gone",
                        s.pid, s.name.c_str(), strerror(errno));
            bool was_stopping = (s.state == State::STOPPING || s.state == State::STUCK);
            s.pid = -1;
            s.exit_code = -1;
            s.stop_stage = 0;
            s.state = was_stopping ? State::STOPPED : State::CRASHED;
            changed = true;
            if (s.pending_restart) {
                s.pending_restart = false;
                startSubsystem(s);
            }
        } else if (s.state == State::STOPPING && now >= s.escalate_at) {
            if (s.stop_stage < 3) {
                s.stop_stage++;
                int sig = (s.stop_stage == 2) ? SIGTERM : SIGKILL;
                RCLCPP_WARN(this->get_logger(), "%s ignoring stop — escalating to %s",
                            s.name.c_str(), sig == SIGTERM ? "SIGTERM" : "SIGKILL");
                signalGroup(s.pid, sig);
                s.escalate_at = now + std::chrono::seconds(3);
            } else {
                RCLCPP_ERROR(this->get_logger(), "%s still unreaped after SIGKILL — likely D-state, "
                             "no further signals; will reap if the kernel releases it", s.name.c_str());
                s.state = State::STUCK;
                changed = true;
            }
        }
    }
    if (changed) publishStatus();
}

void HeartNode::publishStatus(){
    rover_msgs::msg::HeartStatus msg;
    msg.header = rover_utils::createHeader();
    msg.host = my_host_id;
    auto now = std::chrono::steady_clock::now();
    for (const auto& [name, s] : subsystems) {
        State st;
        st.name = name;
        st.state = s.state;
        st.pid = (s.pid > 0) ? s.pid : -1;
        st.exit_code = s.exit_code;
        st.uptime_s = (s.state == State::RUNNING)
            ? std::chrono::duration_cast<std::chrono::seconds>(now - s.started_at).count() : 0;
        msg.subsystems.push_back(st);
    }
    status_pub->publish(msg);
}

void HeartNode::commandCallback(rover_msgs::msg::SubsystemCommand::SharedPtr cmd){
    auto it = subsystems.find(cmd->subsystem_name);
    if (it == subsystems.end()) return;  // another heart's subsystem — shared topic
    SubSystem& s = it->second;
    using Cmd = rover_msgs::msg::SubsystemCommand;
    switch (cmd->action) {
    case Cmd::ACTION_START:   startSubsystem(s); break;
    case Cmd::ACTION_STOP:    stopSubsystem(s); break;
    case Cmd::ACTION_RESTART:
        if (s.state == State::RUNNING) { s.pending_restart = true; stopSubsystem(s); }
        else startSubsystem(s);
        break;
    }
}
