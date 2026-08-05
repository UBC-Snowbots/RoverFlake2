#include "heart/heartNode.h"
#include "rover_utils/include/time_utils.h"
#include <std_msgs/msg/detail/header__struct.hpp>


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv); 
    rclcpp::NodeOptions options;
    // options.automatically_declare_parameters_from_overrides(true); // we don't have to declare params beforehand with this
    // options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<HeartNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

void HeartNode::runSubSystem(SubSystem& subsys){
    // for(const auto &process : monitored_systems[subsystem_name].processes){
        // monitored_systems[subsystem_name].status_label->set_label("STARTING");
        if(subsys.online){
            RCLCPP_ERROR(this->get_logger(), "Subsystem %s already running. Kill first to restart", subsys.name.c_str());
            return;
        }


        pid_t pid = fork();//* once this is called, both the child and parent run the code below, with different outcomes
        if (pid == 0) { //if fork was successful, and I'm the child
            // Child process: replace this process with "ros2 run my_package my_node"
            if (setsid() < 0) {
                perror("setsid failed");
                exit(EXIT_FAILURE);
            }
            pid_t child_sid = getsid(0);
            pid_t child_gpid = getpgrp();
            RCLCPP_INFO(this->get_logger(), "Launched process with PID: %d, GPID: %d, SID: %d\n", pid, child_gpid, child_sid);
            
            execlp("/bin/sh", "/bin/sh", "-c", subsys.exec_command.c_str(),(char *)NULL);
            // if(type == LAUNCHFILE){

            //     execlp("ros2", "ros2", "launch", pkg.c_str(), node_or_launch_file.c_str(), (char *)NULL);
            // }else{
            //     execlp("ros2", "ros2", "run", pkg.c_str(), node_or_launch_file.c_str(), (char *)NULL);
            // }
    
            // If execlp() returns, there was an error. So this code should be blocked if all goes well.
            perror("execlp failed");
            exit(EXIT_FAILURE);
    
        } else if (pid < 0) {
            // fork() error in parent
            perror("fork failed");
        } else {
            // Parent process: fork succeeded
            // 'pid' is the child's PID. We could store it if we want to terminate later.
            pid_t child_sid = pid;
            pid_t child_gpid = pid;

            RCLCPP_INFO(this->get_logger(), "Launched process with PID: %d, GPID: %d, SID: %d\n", pid, child_gpid, child_sid);
            subsys.pid = pid;
            subsys.gpid = pid;
            subsys.sid = pid;
            subsys.online = true;
            // running_child_pids[node_or_launch_file] = pid;
            // monitored_systems[subsystem_name].pid = pid;
            // monitored_systems[subsystem_name].sid = pid;
            // monitored_systems[subsystem_name].gpid = pid;
            // monitored_systems[subsystem_name].online = true;
        }






        // const auto &process = monitored_systems[subsystem_name].process;
        // runChildNode(process.pkg, process.exec, subsystem_name, process.type);
        // Glib::RefPtr<Gtk::StyleContext> context = monitored_systems[subsystem_name].status_label->get_style_context();
        // context->add_class("subsys_ONLINE");
        // context->remove_class("subsys_OFFLINE");
        // monitored_systems[subsystem_name].status_label->set_label("online");


    // }
}

void HeartNode::killSubSystem(SubSystem& subsys){
    if(subsys.online){
        killProcessGroup(subsys.gpid);
        subsys.online = false;
    }else{
        RCLCPP_ERROR(this->get_logger(), "Subsystem %s is already offline", subsys.name.c_str());
    }
}

void HeartNode::killProcessGroup(pid_t pgid) {
    RCLCPP_INFO(this->get_logger(), "Killing process group: %d", pgid);

    pid_t my_pgid = getpgrp();
    if(pgid == my_pgid || pgid == 0){
        RCLCPP_ERROR(this->get_logger(), "Holdup, halted attempt to kill my own proccess group. %d",my_pgid);
        return;
    }
    
    // Send SIGINT to the entire process group, each child will get the signal
    if (killpg(pgid, SIGINT) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error sending SIGINT to process group %d: %s", 
                    pgid, strerror(errno));
    }
    //! Should increase sig to sigterm and whatnot
}



void HeartNode::heartRequestCallback(const rover_msgs::msg::HeartRequest::SharedPtr request){
    // if(subsystems.find(request->subsystem_name) == subsystems.end()){
    if(subsystems.count(request->subsystem_name) == 0){
        RCLCPP_ERROR(this->get_logger(), "Subsystem %s does not exhist", request->subsystem_name.c_str());
        return;
    }

    switch (request->running)
    {
    case true:
        runSubSystem(subsystems[request->subsystem_name]);
        // RCLCPP_INFO(this->get_logger(), "Subsystem %s ENGAGED", request->subsystem_name.c_str());
        break;
    default:
        killSubSystem(subsystems[request->subsystem_name]);
        // RCLCPP_INFO(this->get_logger(), "Subsystem %s KILLED", request->subsystem_name.c_str());
        break;
    }
}



void HeartNode::heartbeat(){
    rover_msgs::msg::HeartRequest msg;
    msg.header = rover_utils::createHeader(); 
    for(const auto &pair : subsystems){
        msg.subsystem_host = my_host_id;
        msg.subsystem_name = pair.first; //or pair.second.name
        msg.index = pair.second.index;
        msg.running = pair.second.online;
        heart_feedback_pub->publish(msg);
    }
}










// std::vector<pid_t> DashboardHMINode::getPidsByName(const std::string &processName, bool verbose)
// {
//     std::vector<pid_t> pids;

//     // Build our command string, e.g.: "pgrep my_node"
//     // -f matches against the entire command line, 
//     // so you might do: "pgrep -f " + processName, depending on your usage
//     const std::string cmd = "pgrep " + processName;

//     // Open a pipe to read the results of pgrep
//     FILE* pipe = popen(cmd.c_str(), "r");
//     if (!pipe) {
//         std::perror("popen failed");
//         return pids;
//     }

//     char buffer[128];
//     while (fgets(buffer, sizeof(buffer), pipe)) {
//         // Each line should contain one PID
//         pid_t pid = static_cast<pid_t>(std::stoi(buffer));
//         pids.push_back(pid);
//     }

//     pclose(pipe);



//     return pids;
// }