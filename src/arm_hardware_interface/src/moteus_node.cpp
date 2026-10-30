#include "moteus_node.h"

void MoteusNode::configureMotors() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        configureMotor(i + 1);

        auto& c = configs_[i];
        publishLog(std::string("# Motor ") + std::to_string(i + 1)
            + " (" + ARM_JOINTS[i].hardware_name + ") configured:"
            + " kp=" + c.format(c.kp)
            + " kd=" + c.format(c.kd)
            + " max_current=" + c.format(c.max_current_A) + "A"
            + " pos=[" + c.format(c.position_min) + ", " + c.format(c.position_max) + "]rev"
            + " max_vel=" + c.format(c.max_velocity) + "rev/s"
            + " gear=" + c.format(c.gear_reduction));
    }
}

void MoteusNode::configureMotor(int motor_id) {
    int controller_id = motor_id - 1;
    
    bool maybe_state = bus_->isConnected(controller_id);
    if (!maybe_state) {
        RCLCPP_WARN(this->get_logger(),
            "Motor %d (%s) NOT CONNECTED. Skipping config.",
            motor_id, ARM_JOINTS[motor_id - 1].hardware_name);
        publishLog("# Motor " + std::to_string(motor_id)
            + " (" + ARM_JOINTS[motor_id - 1].hardware_name + ") NOT CONNECTED — skipped");
        return;
    }

    RCLCPP_INFO(this->get_logger(),
        "Motor %d (%s) detected. Pushing config...",
        motor_id, ARM_JOINTS[motor_id - 1].hardware_name);

    for (const auto& [reg, val] : configs_[motor_id - 1].get_configs()) {
        auto reply = bus_->diagnostic(controller_id, "conf set " + reg + " " + val);
        RCLCPP_INFO(this->get_logger(), "[%d] %s = %s (reply: %s)",
            motor_id, reg.c_str(), val.c_str(), reply.c_str());
    }
}

// ---------------------------------------------------------------------------
// Run loop — runs at 100 Hz
//
// Step 1: merge pending_cmds_ into active_cmds_  (mutex-protected, O(N))
// Step 2: build one CAN frame per motor           (see moteus_protocol.h)
// Step 3: BlockingCycle — send all frames, wait for all replies
// Step 4: decode replies into telem_[]            (see arm_telemetry.h)
// Step 5: safety checks                           (checkFaults, checkAlerts)
// Step 6: publish /arm/moteus_feedback and /joint_states
// ---------------------------------------------------------------------------

// Pending commands from ros -> merge into active commands, but are rejected if there is homing and whatnot.
// Homing is then Checked. 
void MoteusNode::run() {
    // Stage 1-5
    prepareCommands();


    rover_msgs::msg::MoteusArmStatus moteus_ros_msg;
    rover_msgs::msg::ArmCommand arm_feedback_msg;
    arm_feedback_msg.positions.resize(NUM_AXES);
    moteus_ros_msg.status.resize(NUM_AXES);
    moteus_ros_msg.config.resize(NUM_AXES);
    moteus_ros_msg.limit_switches.resize(NUM_AXES);


    // ── Stage 6-7 :) : frames — built ONLY from motor_cmds_ sends them to bus
    // and returns telemetry ───────────────────────
    telem_ = bus_->cycle(motor_cmds_);
    
    // Stage 8
    handleTelemetry();

    // ── Stage 9: safety + publish (unchanged, but read motor_cmds_ for des_) ─
    checkAlerts();
    // ... your existing arm_feedback / MoteusArmStatus / joint_states publish,
    //     with active_cmds_[i]  ->  motor_cmds_[i]  in the des_position/des_velocity lines.
}


// ---------------------------------------------------------------------------
// Startup configuration — pushes MotorConfig values to firmware
// ---------------------------------------------------------------------------

void MoteusNode::publishLog(const std::string& msg) {
    std_msgs::msg::String m;
    m.data = msg;
    config_log_pub_->publish(m);
}

// Zero position of a motor.
void MoteusNode::zero_position(uint8_t index)
{
    int i = static_cast<int>(index);

    bus_->diagnostic(i, "d exact 0");
    RCLCPP_INFO(this->get_logger(),
    "Motor %d (%s) position set to zero (d exact 0)", i + 1, ARM_JOINTS[i].hardware_name);
    publishLog("# Motor " + std::to_string(i + 1)
        + " (" + ARM_JOINTS[i].hardware_name + ") zeroed");
}

void MoteusNode::set_position(uint8_t index, float position_revs)
{
  int i = static_cast<int>(index);

  char cmd[32];
  std::snprintf(cmd, sizeof(cmd), "d exact %f", position_revs);
  bus_->diagnostic(i, cmd);
}


// ---------------------------------------------------------------------------
// Fault detection — edge-triggered (logs only on state change, not every cycle)
// ---------------------------------------------------------------------------

void MoteusNode::checkFaults() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        auto& t = telem_[i];
        if (!t.connected) continue;

        if (t.fault != last_fault_[i]) {
            if (t.fault != 0) {
                RCLCPP_ERROR(this->get_logger(),
                    "Motor %d (%s) FAULT! Code: %d — commands blocked until STOP sent",
                    i + 1, ARM_JOINTS[i].hardware_name, t.fault);
                publishLog("# FAULT motor " + std::to_string(i + 1)
                    + " (" + ARM_JOINTS[i].hardware_name
                    + ") code=" + std::to_string(t.fault));
            } else if (last_fault_[i] != 0) {
                RCLCPP_INFO(this->get_logger(),
                    "Motor %d (%s) fault cleared", i + 1, ARM_JOINTS[i].hardware_name);
                publishLog("# Motor " + std::to_string(i + 1)
                    + " (" + ARM_JOINTS[i].hardware_name + ") fault cleared");
            }
            last_fault_[i] = t.fault;
        }

        if (t.mode != last_mode_[i]) {
            if (t.mode == 1)
                RCLCPP_ERROR(this->get_logger(),
                    "Motor %d (%s) entered FAULT mode", i + 1, ARM_JOINTS[i].hardware_name);
            else if (t.mode == 0 && last_mode_[i] != 0)
                RCLCPP_WARN(this->get_logger(),
                    "Motor %d (%s) is STOPPED (driver disabled)", i + 1, ARM_JOINTS[i].hardware_name);
            last_mode_[i] = t.mode;
        }
    }
}


// ---------------------------------------------------------------------------
// Position limit alerts — warns when approaching configured bounds
// ---------------------------------------------------------------------------

void MoteusNode::checkAlerts() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        auto& t = telem_[i];
        if (!t.connected) continue;

        auto&  c       = configs_[i];
        float  padding = c.position_warn_rev_padding;
        bool near_max  = t.position >= (c.position_max - padding);
        bool near_min  = t.position <= (c.position_min + padding);

        if (near_max || near_min) {
            if (!position_alert_raised_[i]) {
                position_alert_raised_[i] = true;
                const char* which = near_max ? "MAX" : "MIN";
                float limit = near_max ? c.position_max : c.position_min;
                RCLCPP_WARN(this->get_logger(),
                    "Motor %d (%s) near %s position limit! pos=%.3f limit=%.3f (padding=%.3f)",
                    i + 1, ARM_JOINTS[i].hardware_name, which,
                    t.position, limit, padding);
                publishLog("# WARNING motor " + std::to_string(i + 1)
                    + " (" + ARM_JOINTS[i].hardware_name + ") near " + which
                    + " limit: pos=" + std::to_string(t.position)
                    + " limit=" + std::to_string(limit));
            }
        } else {
            position_alert_raised_[i] = false;
        }
    }
}



// ---------------------------------------------------------------------------
// Real-time config update — HMI publishes to /arm/config_update
//
// Applies a single "conf set <register> <value>" to the requested motor and
// updates the in-memory MotorConfig so subsequent feedback messages reflect
// the change.  The moteus firmware applies the value immediately (RAM only —
// not persisted to flash unless you separately run "conf write").
// ---------------------------------------------------------------------------

void MoteusNode::configUpdateCallback(
        const rover_msgs::msg::MoteusConfigUpdate::SharedPtr msg)
{
    int idx = msg->motor_id;
    if (idx < 0 || idx >= NUM_MOTORS) {
        RCLCPP_WARN(this->get_logger(),
            "configUpdate: motor_id %d out of range (0..%d)", idx, NUM_MOTORS - 1);
        return;
    }
    if (msg->register_name.empty() || std::isnan(msg->value)) return;

    const std::string val_str = std::to_string(msg->value);
    const std::string cmd = "conf set " + msg->register_name + " " + val_str;

    RCLCPP_INFO(this->get_logger(),
        "configUpdate motor %d (%s): %s = %s",
        idx + 1, ARM_JOINTS[idx].hardware_name,
        msg->register_name.c_str(), val_str.c_str());
    publishLog("# conf motor " + std::to_string(idx + 1)
        + " (" + ARM_JOINTS[idx].hardware_name + "): "
        + msg->register_name + " = " + val_str);

    bus_->diagnostic(idx, cmd);

    // For max_velocity, the moteus firmware has two registers that must both
    // be updated to cap velocity consistently during motion profile planning.
    if (msg->register_name == "servo.max_velocity") {
        bus_->diagnostic(idx,
            "conf set servo.default_velocity_limit " + val_str);
    } else if (msg->register_name == "servo.default_velocity_limit") {
        bus_->diagnostic(idx,
            "conf set servo.max_velocity " + val_str);
    }

    applyConfigToMemory(idx, msg->register_name, msg->value);
}