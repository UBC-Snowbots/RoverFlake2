# Heart Hardening + HMI Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the heart supervisor observe real child state (reap, escalate, restart) and give the new HMI heart control + staleness indication, per `docs/superpowers/specs/2026-08-02-heart-hardening-design.md`.

**Architecture:** New `SubsystemCommand`/`SubsystemState`/`HeartStatus` messages on `/heart/command` + `/heart/status`; `HeartNode` rewritten around a 250 ms supervise timer (waitpid reap, SIGINT→SIGTERM→SIGKILL escalation, stop-then-start restart); new `SubsystemsModule` GuiModule in rover_hmi_core; shared `StaleMonitor` helper adopted by three panels.

**Tech Stack:** ROS 2 Humble (rclcpp), Qt5 Widgets, pluginlib, colcon in the rover docker container.

## Global Constraints

- Build/verify inside the container: `docker compose --compatibility exec -T rover_ros bash -c "source /opt/ros/humble/setup.bash && cd /RoverFlake2/.claude/worktrees/heart_hardening && <cmd>"` (run compose from the repo root `/home/arhim/Documents/rover/RoverFlake2`).
- This repo has **no automated test infrastructure**. Each task verifies via `colcon build --packages-select <pkg>` plus scripted `ros2` integration checks run in the container (software-only; no hardware is touched). Do not add gtest scaffolding.
- Code style (Aaron): compact, low comment density, no banner/spacer comments, small focused files. Match surrounding idiom.
- Commit style: `feat(pkg): ...` / `fix(pkg): ...`, NO Claude co-author trailer, no session URL trailer.
- Old GTK dashboard (`rover_hmi`) is intentionally untouched and must keep compiling (leave `HeartRequest.msg` and its topics' message types in place).
- Do not modify `motor_status_module.cpp`'s existing watchdog or `DriverProcess` (bench fallback stays as-is; minimizes merge conflicts with a parallel session).
- One aggregated `HeartStatus` per heart per beat (1 Hz) + event publish on state change. No per-subsystem message streams, no chatty logging (bandwidth budget: ~30 Mbps link, cameras dominate).
- HMI-side liveness is judged ONLY by arrival time on the HMI's own steady clock, never by sender header stamps.

---

### Task 1: rover_msgs — SubsystemCommand / SubsystemState / HeartStatus

**Files:**
- Create: `src/rover_msgs/msg/SubsystemCommand.msg`
- Create: `src/rover_msgs/msg/SubsystemState.msg`
- Create: `src/rover_msgs/msg/HeartStatus.msg`
- Modify: `src/rover_msgs/CMakeLists.txt` (msg list, after the `"msg/HeartRequest.msg"` line, currently line 37)

**Interfaces:**
- Produces (used by Tasks 2 and 3): `rover_msgs/msg/subsystem_command.hpp` (`ACTION_START=0, ACTION_STOP=1, ACTION_RESTART=2`), `rover_msgs/msg/subsystem_state.hpp` (`STOPPED=0, RUNNING=1, STOPPING=2, CRASHED=3`), `rover_msgs/msg/heart_status.hpp`.

- [ ] **Step 1: Create the three message files**

`src/rover_msgs/msg/SubsystemCommand.msg`:
```
# HMI -> hearts on /heart/command. Routed by unique subsystem name.
uint8 ACTION_START=0
uint8 ACTION_STOP=1
uint8 ACTION_RESTART=2

string subsystem_name
uint8 action
```

`src/rover_msgs/msg/SubsystemState.msg`:
```
# Observed (reaped) state of one heart-managed child. Never a stored belief.
uint8 STOPPED=0
uint8 RUNNING=1
uint8 STOPPING=2
uint8 CRASHED=3

string name
uint8 state
int32 pid          # -1 when not running
int32 exit_code    # valid in CRASHED/STOPPED after a reap; 128+signal if killed by signal
uint32 uptime_s    # 0 unless RUNNING
```

`src/rover_msgs/msg/HeartStatus.msg`:
```
# One aggregated beat per heart on /heart/status (1 Hz + on any state change).
std_msgs/Header header
string host
SubsystemState[] subsystems
```

- [ ] **Step 2: Register in CMakeLists**

In `src/rover_msgs/CMakeLists.txt`, immediately after `"msg/HeartRequest.msg"`, add:
```cmake
  "msg/SubsystemCommand.msg"
  "msg/SubsystemState.msg"
  "msg/HeartStatus.msg"
```

- [ ] **Step 3: Build and verify**

Run (in container, worktree root): `colcon build --packages-select rover_msgs 2>&1 | tail -3`
Expected: `Finished <<< rover_msgs`, no failures.

Then: `source install/setup.bash && ros2 interface show rover_msgs/msg/HeartStatus`
Expected: shows header, host, `SubsystemState[] subsystems` with nested constants.

- [ ] **Step 4: Commit**

```bash
git add src/rover_msgs
git commit -m "feat(rover_msgs): subsystem command/state + aggregated heart status msgs"
```

---

### Task 2: rover_manager — hardened HeartNode

**Files:**
- Rewrite: `src/rover_manager/src/heart/heartNode.cpp`
- Rewrite: `src/rover_manager/include/heart/heartNode.h`
- Create: `src/rover_manager/config/heart_test.yaml`
- Verify only (no edit): `src/rover_manager/CMakeLists.txt:27-28` already builds `heart_node` with deps `rclcpp rover_utils rover_msgs`.

**Interfaces:**
- Consumes: Task 1 messages.
- Produces: `heart_node` subscribing `rover_msgs/SubsystemCommand` on `/heart/command` (param `command_topic`), publishing `rover_msgs/HeartStatus` on `/heart/status` (param `status_topic`), beat rate from existing `heartbeat_rate` param, host id still derived from node name minus `heart_` prefix, subsystems still from the `subsystems.*` param map (name → shell command). Old `/heart/request` + `/heart/running_subsystems` interface is dropped from the heart (old dashboard retires; its msg type stays in rover_msgs).

- [ ] **Step 1: Rewrite `include/heart/heartNode.h`**

```cpp
//* Per-computer supervisor: starts/stops the subsystems listed in heart.yaml and
//* reports their OBSERVED state (reaped via waitpid, never a stored belief).
#include <rclcpp/rclcpp.hpp>
#include <rover_utils/include/roverCommon.h>
#include <rover_msgs/msg/subsystem_command.hpp>
#include <rover_msgs/msg/heart_status.hpp>
#include <chrono>
#include <sys/types.h>

class HeartNode : public rclcpp::Node
{
public:
    HeartNode();

private:
    using State = rover_msgs::msg::SubsystemState;
    struct SubSystem {
        std::string name;
        std::string exec_command;
        pid_t pid = -1;              // == pgid/sid (child calls setsid)
        uint8_t state = State::STOPPED;
        int exit_code = 0;
        bool pending_restart = false;
        int stop_stage = 0;          // 1=SIGINT sent, 2=SIGTERM, 3=SIGKILL
        std::chrono::steady_clock::time_point started_at, escalate_at;
    };
    std::map<std::string, SubSystem> subsystems;  // ordered -> stable msg order
    std::string my_host_id;

    void startSubsystem(SubSystem& s);
    void stopSubsystem(SubSystem& s);
    void superviseTick();            // 250 ms: reap children, escalate stops
    void publishStatus();
    void commandCallback(rover_msgs::msg::SubsystemCommand::SharedPtr cmd);
    void signalGroup(pid_t pgid, int sig);

    rclcpp::TimerBase::SharedPtr heartbeat_timer, supervise_timer;
    rclcpp::Subscription<rover_msgs::msg::SubsystemCommand>::SharedPtr command_sub;
    rclcpp::Publisher<rover_msgs::msg::HeartStatus>::SharedPtr status_pub;
};
```

- [ ] **Step 2: Rewrite `src/heart/heartNode.cpp`**

```cpp
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
    if(s.state == State::RUNNING || s.state == State::STOPPING){
        RCLCPP_WARN(this->get_logger(), "%s is %s — not starting", s.name.c_str(),
                    s.state == State::RUNNING ? "running" : "stopping");
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
            s.exit_code = WIFEXITED(wstatus) ? WEXITSTATUS(wstatus)
                        : WIFSIGNALED(wstatus) ? 128 + WTERMSIG(wstatus) : -1;
            bool was_stopping = (s.state == State::STOPPING);
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
        } else if (s.state == State::STOPPING && now >= s.escalate_at) {
            s.stop_stage++;
            int sig = (s.stop_stage == 2) ? SIGTERM : SIGKILL;
            RCLCPP_WARN(this->get_logger(), "%s ignoring stop — escalating to %s",
                        s.name.c_str(), sig == SIGTERM ? "SIGTERM" : "SIGKILL");
            signalGroup(s.pid, sig);
            s.escalate_at = now + std::chrono::seconds(3);
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
```

Note the behavioral deltas vs old code: unknown subsystem names are silently ignored (shared topic routing, was ERROR-spam), `SubsystemState` alias `State` is used for constants, and the child logs nothing before exec (the old child-side RCLCPP_INFO raced the parent's logger).

- [ ] **Step 3: Create `src/rover_manager/config/heart_test.yaml`**

```yaml
# Bench/desk test config — no hardware, safe anywhere.
/heart_test_host:
  ros__parameters:
    subsystems:
      sleeper: "sleep 300"
      chatty: "ros2 run demo_nodes_cpp talker"
      stubborn: "bash -c 'trap \"\" INT TERM; sleep 300'"
    heartbeat_rate: 1.0
```

- [ ] **Step 4: Build**

Run: `colcon build --packages-select rover_manager 2>&1 | tail -3`
Expected: `Finished <<< rover_manager`.

- [ ] **Step 5: Scripted integration check (container, no hardware)**

Terminal A (background): `source install/setup.bash && ros2 run rover_manager heart_node --ros-args -r __node:=heart_test_host --params-file src/rover_manager/config/heart_test.yaml`

Checks (each via `ros2 topic pub --once /heart/command rover_msgs/msg/SubsystemCommand "{subsystem_name: X, action: N}"` and `ros2 topic echo /heart/status`):
1. On startup: both subsystems `state: 0` (STOPPED) in ONE aggregated message at 1 Hz.
2. START sleeper (action 0) → `state: 1`, pid > 0, uptime_s counting.
3. `kill -9 <pid>` from a shell → within ~250 ms an event beat with `state: 3` (CRASHED), `exit_code: 137`.
4. START sleeper again (allowed from CRASHED) → RUNNING. STOP (action 1) → `state: 2` briefly, then `state: 0` with `exit_code: 130` (SIGINT kills sleep directly — no escalation expected).
4b. START stubborn → RUNNING. STOP → it ignores SIGINT and SIGTERM; expect two escalation WARN logs (~3 s apart), final `state: 0` with `exit_code: 137` (SIGKILL) after ~6 s.
5. RESTART chatty (action 2) while RUNNING → STOPPING → STOPPED → RUNNING with a new pid, no manual second command.
6. START while RUNNING → warn log, no state change.

Expected: all six behaviors observed. Record actual outputs in the task report.

- [ ] **Step 6: Commit**

```bash
git add src/rover_manager
git commit -m "feat(rover_manager): observed-state heart — reap, escalate, restart, aggregated status"
```

---

### Task 3: rover_hmi_core — Subsystems panel

**Files:**
- Create: `src/rover_hmi_core/include/rover_hmi_core/general/subsystems_module.h`
- Create: `src/rover_hmi_core/src/general/subsystems_module.cpp`
- Modify: `src/rover_hmi_core/plugins.xml` (add class entry)
- Modify: `src/rover_hmi_core/CMakeLists.txt` (add source to `rover_hmi_modules` library, line ~45 block)

**Interfaces:**
- Consumes: `rover_msgs/msg/HeartStatus` on `/heart/status`, publishes `rover_msgs/msg/SubsystemCommand` on `/heart/command` (Task 1 names/constants).
- Produces: pluginlib class `rover_hmi_core/SubsystemsModule`, `name()` "Subsystems", `sectionName()` "General", `layoutHint()` "right".

Threading note: `hmi_host.cpp` drives `rclcpp::spin_some` from a Qt timer, so ROS callbacks run on the GUI thread — touching widgets from the subscription callback is safe and is the established pattern in this package.

- [ ] **Step 1: Write the header**

`include/rover_hmi_core/general/subsystems_module.h`:
```cpp
// Heart control panel: observed per-subsystem state from each per-computer
// heart (/heart/status), start/stop/restart via /heart/command. Host liveness
// is arrival-time on OUR steady clock — sender stamps are never trusted.
#pragma once
#include <QWidget>
#include <QGroupBox>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <map>
#include "rover_hmi_core/gui_module.h"
#include "rover_msgs/msg/heart_status.hpp"
#include "rover_msgs/msg/subsystem_command.hpp"

namespace rover_hmi_core {

class SubsystemsModule : public GuiModule {
public:
    std::string name() const override { return "Subsystems"; }
    std::string sectionName() const override { return "General"; }
    std::string layoutHint() const override { return "right"; }
    QWidget* createWidget(QWidget* parent) override;
    void setNode(rclcpp::Node::SharedPtr node) override;

private:
    struct Row {
        QLabel *chip = nullptr, *uptime = nullptr;
    };
    struct HostGroup {
        QGroupBox* box = nullptr;
        QGridLayout* grid = nullptr;
        std::map<std::string, Row> rows;
        qint64 last_arrival_ms = 0;   // steady clock, stamped on message arrival
    };
    void onStatus(rover_msgs::msg::HeartStatus::SharedPtr msg);
    void sendCommand(const std::string& name, uint8_t action);
    HostGroup& hostGroup(const std::string& host);
    Row& row(HostGroup& g, const std::string& subsystem);
    void applyState(Row& r, const rover_msgs::msg::SubsystemState& s);
    void checkHostsAlive();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<rover_msgs::msg::HeartStatus>::SharedPtr sub_;
    rclcpp::Publisher<rover_msgs::msg::SubsystemCommand>::SharedPtr pub_;
    QWidget* root_ = nullptr;
    QVBoxLayout* hosts_layout_ = nullptr;
    QLabel* waiting_ = nullptr;
    std::map<std::string, HostGroup> hosts_;
};

}  // namespace rover_hmi_core
```

- [ ] **Step 2: Write the implementation**

`src/general/subsystems_module.cpp` — follow the compact style and `theme::` usage of `src/electrical/drivetrain_stop_module.cpp` (read it first for exact theme idiom). Reference implementation:

```cpp
#include "rover_hmi_core/general/subsystems_module.h"
#include "rover_hmi_core/catppuccin.h"
#include <QVBoxLayout>
#include <QTimer>
#include <QElapsedTimer>
#include <pluginlib/class_list_macros.hpp>

namespace rover_hmi_core {
namespace {
QElapsedTimer g_steady;  // steady/monotonic arrival clock
qint64 steadyMs() { if (!g_steady.isValid()) g_steady.start(); return g_steady.elapsed(); }
constexpr qint64 HOST_TIMEOUT_MS = 2500;  // 2.5 beats at 1 Hz
}

void SubsystemsModule::setNode(rclcpp::Node::SharedPtr node) {
    node_ = node;
    sub_ = node_->create_subscription<rover_msgs::msg::HeartStatus>(
        "/heart/status", 10,
        std::bind(&SubsystemsModule::onStatus, this, std::placeholders::_1));
    pub_ = node_->create_publisher<rover_msgs::msg::SubsystemCommand>("/heart/command", 10);
}

QWidget* SubsystemsModule::createWidget(QWidget* parent) {
    root_ = new QWidget(parent);
    auto* outer = new QVBoxLayout(root_);
    waiting_ = new QLabel("⏳ waiting for hearts on /heart/status");
    outer->addWidget(waiting_);
    hosts_layout_ = new QVBoxLayout();
    outer->addLayout(hosts_layout_);
    outer->addStretch();
    auto* alive = new QTimer(root_);
    QObject::connect(alive, &QTimer::timeout, [this]() { checkHostsAlive(); });
    alive->start(500);
    return root_;
}

SubsystemsModule::HostGroup& SubsystemsModule::hostGroup(const std::string& host) {
    auto it = hosts_.find(host);
    if (it != hosts_.end()) return it->second;
    waiting_->hide();
    HostGroup g;
    g.box = new QGroupBox(QString::fromStdString(host));
    g.grid = new QGridLayout(g.box);
    hosts_layout_->addWidget(g.box);
    return hosts_.emplace(host, g).first->second;
}

SubsystemsModule::Row& SubsystemsModule::row(HostGroup& g, const std::string& name) {
    auto it = g.rows.find(name);
    if (it != g.rows.end()) return it->second;
    const int r = g.rows.size();
    Row rw;
    g.grid->addWidget(new QLabel(QString::fromStdString(name)), r, 0);
    rw.chip = new QLabel("—");
    rw.chip->setAlignment(Qt::AlignCenter);
    g.grid->addWidget(rw.chip, r, 1);
    rw.uptime = new QLabel("");
    g.grid->addWidget(rw.uptime, r, 2);
    const struct { const char* label; uint8_t action; } btns[] = {
        {"▶", rover_msgs::msg::SubsystemCommand::ACTION_START},
        {"■", rover_msgs::msg::SubsystemCommand::ACTION_STOP},
        {"↻", rover_msgs::msg::SubsystemCommand::ACTION_RESTART},
    };
    for (int i = 0; i < 3; i++) {
        auto* b = new QPushButton(btns[i].label);
        b->setFixedWidth(32);
        const uint8_t action = btns[i].action;
        QObject::connect(b, &QPushButton::clicked,
                         [this, name, action]() { sendCommand(name, action); });
        g.grid->addWidget(b, r, 3 + i);
    }
    return g.rows.emplace(name, rw).first->second;
}

void SubsystemsModule::applyState(Row& rw, const rover_msgs::msg::SubsystemState& s) {
    using St = rover_msgs::msg::SubsystemState;
    switch (s.state) {
    case St::RUNNING:
        rw.chip->setText("RUNNING");
        rw.chip->setStyleSheet(QString("color:%1;font-weight:bold;").arg(theme::Green));
        rw.uptime->setText(QString("%1 s").arg(s.uptime_s));
        break;
    case St::STOPPING:
        rw.chip->setText("STOPPING");
        rw.chip->setStyleSheet(QString("color:%1;font-weight:bold;").arg(theme::Yellow));
        rw.uptime->setText("");
        break;
    case St::CRASHED:
        rw.chip->setText(QString("CRASHED (%1)").arg(s.exit_code));
        rw.chip->setStyleSheet(QString("color:%1;font-weight:bold;").arg(theme::Red));
        rw.uptime->setText("");
        break;
    default:
        rw.chip->setText("STOPPED");
        rw.chip->setStyleSheet(QString("color:%1;").arg(theme::Overlay));
        rw.uptime->setText("");
    }
}

void SubsystemsModule::onStatus(rover_msgs::msg::HeartStatus::SharedPtr msg) {
    HostGroup& g = hostGroup(msg->host);
    g.last_arrival_ms = steadyMs();
    g.box->setTitle(QString::fromStdString(msg->host));
    g.box->setStyleSheet("");
    for (const auto& s : msg->subsystems) applyState(row(g, s.name), s);
}

void SubsystemsModule::checkHostsAlive() {
    for (auto& [host, g] : hosts_) {
        if (steadyMs() - g.last_arrival_ms <= HOST_TIMEOUT_MS) continue;
        g.box->setTitle(QString("%1 — ✖ HEART OFFLINE").arg(QString::fromStdString(host)));
        g.box->setStyleSheet(QString("QGroupBox{color:%1;}").arg(theme::Red));
        for (auto& [n, rw] : g.rows) {
            rw.chip->setText("?");
            rw.chip->setStyleSheet(QString("color:%1;").arg(theme::Overlay));
        }
    }
}

void SubsystemsModule::sendCommand(const std::string& name, uint8_t action) {
    rover_msgs::msg::SubsystemCommand cmd;
    cmd.subsystem_name = name;
    cmd.action = action;
    pub_->publish(cmd);
}

}  // namespace rover_hmi_core
PLUGINLIB_EXPORT_CLASS(rover_hmi_core::SubsystemsModule, rover_hmi_core::GuiModule)
```

IMPORTANT: check `catppuccin.h` for the real constant names (`theme::Green` / `theme::Yellow` / `theme::Red` / `theme::Overlay` are assumed — substitute whatever the header actually defines, matching how other modules color labels).

- [ ] **Step 3: Register the plugin**

In `src/rover_hmi_core/plugins.xml`, add inside the `<library>` element (near the other General-section modules):
```xml
  <class name="rover_hmi_core/SubsystemsModule"
         type="SubsystemsModule"
         base_class_type="rover_hmi_core::GuiModule">
    <description>Per-computer heart supervisor status and subsystem start/stop/restart</description>
  </class>
```

In `src/rover_hmi_core/CMakeLists.txt`, add `src/general/subsystems_module.cpp` to the `rover_hmi_modules` `add_library(...)` source list.

- [ ] **Step 4: Build**

Run: `colcon build --packages-select rover_hmi_core 2>&1 | tail -3`
Expected: `Finished <<< rover_hmi_core`.

- [ ] **Step 5: Integration check against the Task 2 heart**

With the test heart running (Task 2 Step 5 command) — HMI needs a display; if `$DISPLAY` is unavailable in the container this check is deferred to Aaron with exact commands in the task report:
`source install/setup.bash && ros2 run rover_hmi_core rover_hmi`
Expected: Subsystems tile shows host `test_host`, rows `sleeper`/`chatty` with grey STOPPED chips; ▶ starts (green + uptime), ↻ restarts, ■ stops; killing the heart (Ctrl-C in terminal A) turns the group title red "HEART OFFLINE" within ~3 s and chips to "?".

- [ ] **Step 6: Commit**

```bash
git add src/rover_hmi_core
git commit -m "feat(rover_hmi_core): subsystems panel — heart status + start/stop/restart"
```

---

### Task 4: rover_hmi_core — StaleMonitor + adoption in three panels

**Files:**
- Create: `src/rover_hmi_core/include/rover_hmi_core/stale_monitor.h` (header-only)
- Modify: `src/rover_hmi_core/src/electrical/wheel_telemetry_module.cpp` (sub at line ~154)
- Modify: `src/rover_hmi_core/src/electrical/power_summary_module.cpp` (sub at line ~316)
- Modify: `src/rover_hmi_core/src/science/science_analysis_module.cpp` (sub at line ~461)

Do NOT touch `motor_status_module.cpp` — its hand-rolled watchdog stays (parallel-session merge safety).

**Interfaces:**
- Produces: `rover_hmi_core::StaleMonitor` with `void stamp()`, `void attach(QWidget* parent, int threshold_ms, std::function<void(bool stale)> on_change)`.

- [ ] **Step 1: Write `include/rover_hmi_core/stale_monitor.h`**

```cpp
// Arrival-time staleness watchdog: call stamp() in your subscription callback,
// attach() once at widget creation. on_change(true) fires when data stops
// arriving (only after data has flowed at least once), on_change(false) when
// it resumes. Steady clock; sender stamps are never consulted.
#pragma once
#include <QElapsedTimer>
#include <QTimer>
#include <QWidget>
#include <functional>

namespace rover_hmi_core {

class StaleMonitor {
public:
    void stamp() {
        if (!clock_.isValid()) clock_.start();
        last_ms_ = clock_.elapsed();
        seen_ = true;
    }
    void attach(QWidget* parent, int threshold_ms, std::function<void(bool)> on_change) {
        if (!clock_.isValid()) clock_.start();
        auto* t = new QTimer(parent);
        QObject::connect(t, &QTimer::timeout, [this, threshold_ms, cb = std::move(on_change)]() {
            const bool stale = seen_ && (clock_.elapsed() - last_ms_ > threshold_ms);
            if (stale != stale_) { stale_ = stale; cb(stale); }
        });
        t->start(500);
    }
private:
    QElapsedTimer clock_;
    qint64 last_ms_ = 0;
    bool seen_ = false, stale_ = false;
};

}  // namespace rover_hmi_core
```

- [ ] **Step 2: Adopt in Wheel Telemetry**

In `wheel_telemetry_module.cpp`: add member `StaleMonitor stale_;` (+ include), call `stale_.stamp();` first line of the `/drivetrain/wheel_states` callback (sub created at ~line 154), and in `createWidget` after the table is built:
```cpp
stale_.attach(root, 1500, [this](bool stale) {
    banner_->setText(stale ? "✖ WHEEL TELEMETRY STALE — /drivetrain/wheel_states stopped" : "");
    banner_->setVisible(stale);
});
```
The module may not have a banner label yet — add a hidden `QLabel* banner_` at the top of its layout styled like the arm panel's banner (see `motor_status_module.cpp` `setBanner` colors: `theme::Red` on `#2a0d0d`). Read the module first and integrate with its actual member names/layout.

- [ ] **Step 3: Adopt in Power Summary**

Same pattern in `power_summary_module.cpp`: `stale_.stamp()` in the `/power/status` callback (sub at ~line 316), banner "✖ POWER DATA STALE — /power/status stopped", threshold 3000 ms (power status is slower-rate than wheel telemetry — check the publisher rate in the module comments/code and pick 2.5× its period, min 1500 ms).

- [ ] **Step 4: Adopt in Science Analysis**

Same pattern in `science_analysis_module.cpp`: `stale_.stamp()` in the `/science/sensor_data` callback (sub at ~line 461), banner "✖ SENSOR DATA STALE — /science/sensor_data stopped", threshold 3000 ms. Note: this topic currently has NO publisher on the rover (known); the seen_-gate means the panel shows no false banner when nothing has ever arrived — verify that stays true.

- [ ] **Step 5: Build + check**

Run: `colcon build --packages-select rover_hmi_core 2>&1 | tail -3` — expect `Finished`.
Runtime: with the HMI up (or deferred to Aaron), `ros2 topic pub -r 10 /drivetrain/wheel_states rover_msgs/msg/WheelStates "{}"`, stop the pub → banner appears within ~2 s; resume → banner clears.

- [ ] **Step 6: Commit**

```bash
git add src/rover_hmi_core
git commit -m "feat(rover_hmi_core): shared StaleMonitor; stale banners for wheels, power, science"
```

---

### Task 5: heart.yaml production config — BLOCKED on Aaron

**Files:**
- Modify: `src/rover_manager/config/heart.yaml`

Blocked until Aaron supplies the real per-machine subsystem list. When it arrives:

- [ ] **Step 1: Replace placeholder subsystems**

Remove `banana`/`apple`/`cucumber`/`meow`; keep/confirm `arm_interface: "ros2 launch arm_hardware_interface arm_hardware_bringup.launch.py"`; replace `drive_control: "ros2 run joy joy_node"` and `drive_interface` with Aaron's real commands; add whatever else he lists per machine.

- [ ] **Step 2: Update dashboard params section**

The `/dashboard_hmi_node` block (old dashboard) stays as-is (retiring). Add `command_topic`/`status_topic` to both heart sections ONLY if non-default values are wanted; defaults already match `/heart/command` + `/heart/status`.

- [ ] **Step 3: Verify + commit**

`colcon build --packages-select rover_manager` (config installs), then Aaron reloads the systemd hearts on the rover machines (`sudo systemctl daemon-reload && systemctl restart rover_heart_*` — his hands, per the yaml header warning).
```bash
git add src/rover_manager/config/heart.yaml
git commit -m "feat(rover_manager): production heart.yaml subsystem lists"
```
