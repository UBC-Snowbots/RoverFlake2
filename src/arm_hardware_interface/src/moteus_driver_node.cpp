#include "moteus_driver_node.h"
#include "axis_5_6_differential.h"
#include <cmath>

// =============================================================================
// MoteusDriverNode implementation
//
// For protocol details, units, and data field contents see moteus_protocol.h.
// For joint mapping and direction signs see motor_addressing.h.
// For per-motor PID/limits see motor_config.h.
// =============================================================================

MoteusDriverNode::MoteusDriverNode() : Node("moteus_driver") {
    transport_ = mot::Controller::MakeSingletonTransport({});
    configs_   = get_arm_configuration();

    for (int id = 1; id <= NUM_MOTORS; id++) {
        mot::Controller::Options opts;
        opts.id = id;
        controllers_.push_back(std::make_shared<mot::Controller>(opts));
    }

    auto qos = rclcpp::QoS(1).reliable().durability_volatile();

    feedback_pub_  = this->create_publisher<rover_msgs::msg::MoteusArmStatus>("/arm/moteus_feedback", qos);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);
    config_log_pub_  = this->create_publisher<std_msgs::msg::String>("/arm/config_log", qos);

    command_sub_ = this->create_subscription<rover_msgs::msg::ArmCommand>(
        "/arm/command", qos,
        std::bind(&MoteusDriverNode::commandCallback, this, std::placeholders::_1));

    configureMotors();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&MoteusDriverNode::poll, this));

    RCLCPP_INFO(this->get_logger(),
        "Moteus driver started: polling %d motors at 100 Hz", NUM_MOTORS);
}


// ---------------------------------------------------------------------------
// Startup configuration — pushes MotorConfig values to firmware
// ---------------------------------------------------------------------------

void MoteusDriverNode::publishLog(const std::string& msg) {
    std_msgs::msg::String m;
    m.data = msg;
    config_log_pub_->publish(m);
}

void MoteusDriverNode::configureMotors() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        configureMotor(i + 1, *controllers_[i]);

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

void MoteusDriverNode::configureMotor(int motor_id, mot::Controller& ctrl) {
    auto maybe_state = ctrl.SetQuery();
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
        auto reply = ctrl.DiagnosticCommand("conf set " + reg + " " + val);
        RCLCPP_INFO(this->get_logger(), "[%d] %s = %s (reply: %s)",
            motor_id, reg.c_str(), val.c_str(), reply.c_str());
    }
}


// ---------------------------------------------------------------------------
// Command callback — ROS topic → pending_cmds_[]
// (See arm_commands.h for command code definitions and MotorCommand struct.)
// ---------------------------------------------------------------------------

void MoteusDriverNode::commandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    char cmd_type = msg->cmd_type;

    if (cmd_type == CMD_STOP) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            pending_cmds_[i].active  = true;
            pending_cmds_[i].is_stop = true;
        }
        RCLCPP_INFO(this->get_logger(), "Command: STOP ALL");
        return;
    }

    if (cmd_type == CMD_ABS_POS) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            double pos = (i < (int)msg->positions.size())  ? msg->positions[i]  : NAN;
            double vel = (i < (int)msg->velocities.size()) ? msg->velocities[i] : NAN;
            if (std::isnan(pos) && std::isnan(vel)) continue;

            if (telem_[i].fault != 0) {
                RCLCPP_WARN(this->get_logger(),
                    "BLOCKED position cmd to motor %d (%s) — fault %d active. "
                    "Send STOP first to clear.",
                    i + 1, ARM_JOINTS[i].hardware_name, telem_[i].fault);
                publishLog("# BLOCKED cmd to motor " + std::to_string(i + 1)
                    + " (" + ARM_JOINTS[i].hardware_name
                    + ") — fault " + std::to_string(telem_[i].fault));
                continue;
            }

            pending_cmds_[i].active    = true;
            pending_cmds_[i].is_stop   = false;
            pending_cmds_[i].position  = pos;
            pending_cmds_[i].velocity  = vel;
            pending_cmds_[i].max_torque = NAN;
        }
        return;
    }

    if (cmd_type == CMD_ABS_VEL) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            double vel = (i < (int)msg->velocities.size()) ? msg->velocities[i] : NAN;
            if (std::isnan(vel)) continue;

            if (vel != 0.0 && telem_[i].fault != 0) {
                RCLCPP_WARN(this->get_logger(),
                    "BLOCKED velocity cmd to motor %d (%s) — fault %d active. "
                    "Send STOP first to clear.",
                    i + 1, ARM_JOINTS[i].hardware_name, telem_[i].fault);
                publishLog("# BLOCKED cmd to motor " + std::to_string(i + 1)
                    + " (" + ARM_JOINTS[i].hardware_name
                    + ") — fault " + std::to_string(telem_[i].fault));
                continue;
            }

            pending_cmds_[i].active  = true;
            if (vel == 0.0) {
                pending_cmds_[i].is_stop = true;
            } else {
                pending_cmds_[i].is_stop  = false;
                pending_cmds_[i].position = NAN;
                pending_cmds_[i].velocity = vel;
                pending_cmds_[i].max_torque = NAN;
            }
        }
        return;
    }

    if (cmd_type == CMD_ZERO) {
        // positions[] used as a flag: non-NaN entry → zero that motor
        for (int i = 0; i < NUM_MOTORS; i++) {
            double flag = (i < (int)msg->positions.size()) ? msg->positions[i] : NAN;
            if (!std::isnan(flag)) {
                pending_cmds_[i].active  = true;
                pending_cmds_[i].is_zero = true;
            }
        }
        return;
    }

    RCLCPP_WARN(this->get_logger(), "Unknown cmd_type: '%c' (%d)", cmd_type, (int)cmd_type);
}


// ---------------------------------------------------------------------------
// Poll loop — runs at 10 Hz
//
// Step 1: merge pending_cmds_ into active_cmds_  (mutex-protected, O(N))
// Step 2: build one CAN frame per motor           (see moteus_protocol.h)
// Step 3: BlockingCycle — send all frames, wait for all replies
// Step 4: decode replies into telem_[]            (see arm_telemetry.h)
// Step 5: safety checks                           (checkFaults, checkAlerts)
// Step 6: publish /arm/moteus_feedback and /joint_states
// ---------------------------------------------------------------------------

void MoteusDriverNode::poll() {
    // Step 1 — merge
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (pending_cmds_[i].active)
                active_cmds_[i] = pending_cmds_[i];
        }
        pending_cmds_ = {};
    }

    // Step 2a — "d exact 0" zero commands (diagnostic channel, separate from BlockingCycle)
    // This resets the position counter to 0 at the current physical position — no movement.
    for (int i = 0; i < NUM_MOTORS; i++) {
        auto& cmd = active_cmds_[i];
        if (cmd.active && cmd.is_zero) {
            controllers_[i]->DiagnosticCommand("d exact 0");
            RCLCPP_INFO(this->get_logger(),
                "Motor %d (%s) zeroed (d exact 0)", i + 1, ARM_JOINTS[i].hardware_name);
            publishLog("# Motor " + std::to_string(i + 1)
                + " (" + ARM_JOINTS[i].hardware_name + ") zeroed");
            cmd.active = false;  // one-shot; next cycle falls through to MakeQuery
        }
    }

    // Step 2b — apply differential wrist transform for axes 5 & 6 (indices 4 & 5)
    // The wrist is mechanically coupled: joint-space commands must be converted
    // to motor-space commands before sending.  See axis_5_6_differential.h.
    if (active_cmds_[4].active && active_cmds_[5].active &&
        !active_cmds_[4].is_stop && !active_cmds_[5].is_stop)
    {
        float m5, m6;
        differential_drive(
            static_cast<float>(active_cmds_[4].velocity),
            static_cast<float>(active_cmds_[5].velocity),
            m5, m6);
        active_cmds_[4].velocity = m5;
        active_cmds_[5].velocity = m6;
    }

    // Step 2c — build frames
    // (See moteus_protocol.h for what goes in the data field of each type.)
    std::vector<mot::CanFdFrame> frames;
    for (int i = 0; i < NUM_MOTORS; i++) {
        auto& cmd = active_cmds_[i];
        if (cmd.active && cmd.is_stop) {
            frames.push_back(MoteusProtocol::makeStopFrame(*controllers_[i]));
            cmd.active = false;  // stop is one-shot; next cycle falls through to query
        } else if (cmd.active) {
            frames.push_back(MoteusProtocol::makePositionFrame(
                *controllers_[i], cmd.position, cmd.velocity, cmd.max_torque));
        } else {
            frames.push_back(MoteusProtocol::makeQueryFrame(*controllers_[i]));
        }
    }

    // Step 3 — send + receive
    std::vector<mot::CanFdFrame> replies;
    transport_->BlockingCycle(frames.data(), frames.size(), &replies);

    // Step 4 — decode replies
    // Reset connected flags; only set true for motors that replied this cycle.
    for (auto& t : telem_) t.connected = false;

    for (const auto& frame : replies) {
        int id = frame.source;
        if (id < 1 || id > NUM_MOTORS) continue;

        auto r = MoteusProtocol::parseReply(frame);
        auto& t = telem_[id - 1];
        t.position    = r.position;
        t.velocity    = r.velocity;
        t.torque      = r.torque;
        t.voltage     = r.voltage;
        t.temperature = r.temperature;
        t.mode        = static_cast<int>(r.mode);
        t.fault       = static_cast<int>(r.fault);
        t.connected   = true;
    }

    // Step 5 — safety
    checkFaults();
    checkAlerts();

    // Step 6a — /arm/moteus_feedback
    rover_msgs::msg::MoteusArmStatus status_msg;
    status_msg.status.resize(NUM_MOTORS);
    status_msg.config.resize(NUM_MOTORS);

    for (int i = 0; i < NUM_MOTORS; i++) {
        auto& c   = configs_[i];
        auto& cfg = status_msg.config[i];
        cfg.max_acceleration  = c.max_acceleration;
        cfg.max_velocity      = c.max_velocity;
        cfg.max_position      = c.position_max;
        cfg.min_position      = c.position_min;
        cfg.kp                = c.kp;
        cfg.ki                = c.ki;
        cfg.kd                = c.kd;
        cfg.max_current_amps  = c.max_current_A;
        cfg.max_voltage_volts = c.max_voltage;
        cfg.max_power_watts   = c.max_power_W;
        cfg.gear_reduction    = c.gear_reduction;

        auto& t = telem_[i];
        auto& s = status_msg.status[i];
        s.curr_position      = t.position;
        s.curr_velocity      = t.velocity;
        s.curr_torque        = t.torque;
        s.curr_voltage_volts = t.voltage;
        s.driver_temp_degreesc = t.temperature;
        s.moteus_mode        = static_cast<int16_t>(t.mode);
        s.moteus_fault       = static_cast<int16_t>(t.fault);
    }
    feedback_pub_->publish(status_msg);

    // Step 6b — /joint_states (consumed by robot_state_publisher → RViz2)
    // Unit conversion via motorRevToJointRad() — see motor_addressing.h.
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name.resize(NUM_MOTORS + NUM_GRIPPER_JOINTS);
    js.position.resize(NUM_MOTORS + NUM_GRIPPER_JOINTS);
    js.velocity.resize(NUM_MOTORS + NUM_GRIPPER_JOINTS);

    for (int i = 0; i < NUM_MOTORS; i++) {
        js.name[i]     = ARM_JOINTS[i].urdf_joint_name;
        js.position[i] = motorRevToJointRad(i, telem_[i].position);
        js.velocity[i] = motorRevPerSecToJointRadPerSec(i, telem_[i].velocity);
    }
    for (int g = 0; g < NUM_GRIPPER_JOINTS; g++) {
        js.name[NUM_MOTORS + g]     = GRIPPER_JOINT_NAMES[g];
        js.position[NUM_MOTORS + g] = 0.0;
        js.velocity[NUM_MOTORS + g] = 0.0;
    }
    joint_state_pub_->publish(js);
}


// ---------------------------------------------------------------------------
// Fault detection — edge-triggered (logs only on state change, not every cycle)
// ---------------------------------------------------------------------------

void MoteusDriverNode::checkFaults() {
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

void MoteusDriverNode::checkAlerts() {
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


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoteusDriverNode>());
    rclcpp::shutdown();
    return 0;
}
