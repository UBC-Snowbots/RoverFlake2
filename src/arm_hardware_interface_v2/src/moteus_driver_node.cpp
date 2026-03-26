#include "moteus_driver_node.h"
#include <cmath>

// URDF joint names in motor order (1-6)
static const std::string URDF_JOINT_NAMES[] = {
    "shoulder_joint", "link_1_joint", "link1_link2",
    "a4_rotation", "a5_rotation", "a6_rotation"
};

// Assumed initial positions (radians) — since we have no homing,
// we treat these as the zero offset for converting motor revolutions
// to URDF joint angles:  joint_rad = initial + motor_rev * 2π
static const double INITIAL_POSITIONS[] = {
    -1.57, -1.57, 0.9, 0.0, 1.2, 0.0
};

MoteusDriverNode::MoteusDriverNode() : Node("moteus_driver") {
    transport_ = mot::Controller::MakeSingletonTransport({});
    configs_ = get_arm_configuration();

    for (int id = 1; id <= NUM_MOTORS; id++) {
        mot::Controller::Options opts;
        opts.id = id;
        controllers_.push_back(std::make_shared<mot::Controller>(opts));
    }

    auto qos = rclcpp::QoS(1).reliable().durability_volatile();

    feedback_pub_ = this->create_publisher<rover_msgs::msg::MoteusArmStatus>(
        "/arm/moteus_feedback", qos);

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", qos);

    config_log_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/arm/config_log", qos);

    command_sub_ = this->create_subscription<rover_msgs::msg::ArmCommand>(
        "/arm/command", qos,
        std::bind(&MoteusDriverNode::commandCallback, this, std::placeholders::_1));

    configureMotors();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MoteusDriverNode::poll, this));

    RCLCPP_INFO(this->get_logger(),
        "Moteus driver started: polling %d motors at 10 Hz", NUM_MOTORS);
}

void MoteusDriverNode::publishLog(const std::string& msg) {
    std_msgs::msg::String log_msg;
    log_msg.data = msg;
    config_log_pub_->publish(log_msg);
}

void MoteusDriverNode::configureMotors() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        configureMotor(i + 1, *controllers_[i]);

        auto& c = configs_[i];
        publishLog(std::string("# Motor ") + std::to_string(i + 1)
            + " (" + JOINT_NAMES[i] + ") configured:"
            + " kp=" + c.format(c.kp)
            + " kd=" + c.format(c.kd)
            + " max_current=" + c.format(c.max_current_A) + "A"
            + " pos=[" + c.format(c.position_min) + ", " + c.format(c.position_max) + "]rev"
            + " max_vel=" + c.format(c.max_velocity) + "rev/s"
            + " gear=" + c.format(c.gear_reduction));
    }
}

void MoteusDriverNode::configureMotor(int motor_id, mot::Controller& controller) {
    auto maybe_state = controller.SetQuery();
    if (!maybe_state) {
        RCLCPP_WARN(this->get_logger(),
            "Motor %d NOT CONNECTED. Skipping config.", motor_id);
        publishLog("# Motor " + std::to_string(motor_id) + " NOT CONNECTED — skipped");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Motor %d detected. Pushing config...", motor_id);

    auto settings = configs_[motor_id - 1].get_configs();
    for (const auto& pair : settings) {
        std::string cmd = "conf set " + pair.first + " " + pair.second;
        auto reply = controller.DiagnosticCommand(cmd);
        RCLCPP_INFO(this->get_logger(), "[%d] %s = %s (reply: %s)",
            motor_id, pair.first.c_str(), pair.second.c_str(), reply.c_str());
    }
}

// ---------------------------------------------------------------------------
// Command callback — with fault blocking
// ---------------------------------------------------------------------------

void MoteusDriverNode::commandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    char cmd_type = msg->cmd_type;

    if (cmd_type == CMD_STOP) {
        // Stop is always allowed — even during faults
        for (int i = 0; i < NUM_MOTORS; i++) {
            pending_cmds_[i].active = true;
            pending_cmds_[i].is_stop = true;
        }
        RCLCPP_INFO(this->get_logger(), "Command: STOP ALL");
        return;
    }

    if (cmd_type == CMD_ABS_POS) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            double pos = (i < (int)msg->positions.size()) ? msg->positions[i] : NAN;
            double vel = (i < (int)msg->velocities.size()) ? msg->velocities[i] : NAN;
            if (std::isnan(pos) && std::isnan(vel)) continue;

            // Block commands to faulted motors
            if (telem_[i].fault != 0) {
                RCLCPP_WARN(this->get_logger(),
                    "BLOCKED position cmd to motor %d — fault %d active. "
                    "Send STOP first to clear.", i + 1, telem_[i].fault);
                publishLog("# BLOCKED cmd to motor " + std::to_string(i + 1)
                    + " — fault " + std::to_string(telem_[i].fault));
                continue;
            }

            pending_cmds_[i].active = true;
            pending_cmds_[i].is_stop = false;
            pending_cmds_[i].position = pos;
            pending_cmds_[i].velocity = vel;
            pending_cmds_[i].max_torque = NAN;
        }
        return;
    }

    if (cmd_type == CMD_ABS_VEL) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            double vel = (i < (int)msg->velocities.size()) ? msg->velocities[i] : NAN;
            if (std::isnan(vel)) continue;

            // Zero velocity (stop) is always allowed
            if (vel != 0.0 && telem_[i].fault != 0) {
                RCLCPP_WARN(this->get_logger(),
                    "BLOCKED velocity cmd to motor %d — fault %d active. "
                    "Send STOP first to clear.", i + 1, telem_[i].fault);
                publishLog("# BLOCKED cmd to motor " + std::to_string(i + 1)
                    + " — fault " + std::to_string(telem_[i].fault));
                continue;
            }

            pending_cmds_[i].active = true;
            if (vel == 0.0) {
                pending_cmds_[i].is_stop = true;
            } else {
                pending_cmds_[i].is_stop = false;
                pending_cmds_[i].position = NAN;
                pending_cmds_[i].velocity = vel;
                pending_cmds_[i].max_torque = NAN;
            }
        }
        return;
    }

    RCLCPP_WARN(this->get_logger(), "Unknown cmd_type: '%c' (%d)", cmd_type, (int)cmd_type);
}

// ---------------------------------------------------------------------------
// Poll loop
// ---------------------------------------------------------------------------

void MoteusDriverNode::poll() {
    // Merge new commands into the persistent active_cmds_ array.
    // Commands persist until overridden or stopped — this keeps the
    // moteus position stream alive so the motor doesn't timeout-fault.
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (pending_cmds_[i].active) {
                active_cmds_[i] = pending_cmds_[i];
            }
        }
        pending_cmds_ = {};
    }

    std::vector<mot::CanFdFrame> frames;
    for (int i = 0; i < NUM_MOTORS; i++) {
        auto& cmd = active_cmds_[i];
        if (cmd.active && cmd.is_stop) {
            frames.push_back(controllers_[i]->MakeStop());
            cmd.active = false;  // stop is one-shot, then revert to query
        } else if (cmd.active) {
            mot::PositionMode::Command pos_cmd;
            pos_cmd.position = cmd.position;
            pos_cmd.velocity = cmd.velocity;
            if (!std::isnan(cmd.max_torque))
                pos_cmd.maximum_torque = cmd.max_torque;
            frames.push_back(controllers_[i]->MakePosition(pos_cmd));
        } else {
            frames.push_back(controllers_[i]->MakeQuery());
        }
    }

    std::vector<mot::CanFdFrame> replies;
    transport_->BlockingCycle(frames.data(), frames.size(), &replies);

    // Update local telemetry
    for (auto& t : telem_) t.connected = false;

    for (const auto& frame : replies) {
        int id = frame.source;
        if (id < 1 || id > NUM_MOTORS) continue;

        auto r = mot::Query::Parse(frame.data, frame.size);
        auto& t = telem_[id - 1];
        t.position = r.position;
        t.velocity = r.velocity;
        t.torque = r.torque;
        t.voltage = r.voltage;
        t.temperature = r.temperature;
        t.mode = static_cast<int>(r.mode);
        t.fault = static_cast<int>(r.fault);
        t.connected = true;
    }

    // Check safety conditions
    checkFaults();
    checkAlerts();

    // Build and publish feedback message
    rover_msgs::msg::MoteusArmStatus status_msg;
    status_msg.status.resize(NUM_MOTORS);
    status_msg.config.resize(NUM_MOTORS);

    for (int i = 0; i < NUM_MOTORS; i++) {
        auto& c = configs_[i];
        auto& cfg = status_msg.config[i];
        cfg.max_acceleration = c.max_acceleration;
        cfg.max_velocity = c.max_velocity;
        cfg.max_position = c.position_max;
        cfg.min_position = c.position_min;
        cfg.kp = c.kp;
        cfg.ki = c.ki;
        cfg.kd = c.kd;
        cfg.max_current_amps = c.max_current_A;
        cfg.max_voltage_volts = c.max_voltage;
        cfg.max_power_watts = c.max_power_W;
        cfg.gear_reduction = c.gear_reduction;

        auto& t = telem_[i];
        auto& s = status_msg.status[i];
        s.curr_position = t.position;
        s.curr_velocity = t.velocity;
        s.curr_torque = t.torque;
        s.curr_voltage_volts = t.voltage;
        s.driver_temp_degreesc = t.temperature;
        s.moteus_mode = static_cast<int16_t>(t.mode);
        s.moteus_fault = static_cast<int16_t>(t.fault);
    }

    feedback_pub_->publish(status_msg);

    // Publish /joint_states for robot_state_publisher / RViz2
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name.resize(NUM_MOTORS + 2);  // 6 arm joints + 2 gripper
    js.position.resize(NUM_MOTORS + 2);
    js.velocity.resize(NUM_MOTORS + 2);

    for (int i = 0; i < NUM_MOTORS; i++) {
        js.name[i] = URDF_JOINT_NAMES[i];
        // Convert: motor position (revolutions) -> radians, offset by initial position
        js.position[i] = INITIAL_POSITIONS[i] + (telem_[i].position * 2.0 * M_PI);
        js.velocity[i] = telem_[i].velocity * 2.0 * M_PI;
    }

    // Gripper fingers (no motor data, publish as static 0)
    js.name[NUM_MOTORS]     = "finger_left_joint";
    js.name[NUM_MOTORS + 1] = "finger_right_joint";
    js.position[NUM_MOTORS]     = 0.0;
    js.position[NUM_MOTORS + 1] = 0.0;
    js.velocity[NUM_MOTORS]     = 0.0;
    js.velocity[NUM_MOTORS + 1] = 0.0;

    joint_state_pub_->publish(js);
}

// ---------------------------------------------------------------------------
// Fault detection — logs on state change, not every cycle
// ---------------------------------------------------------------------------

void MoteusDriverNode::checkFaults() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        auto& t = telem_[i];
        if (!t.connected) continue;

        // Fault state changed
        if (t.fault != last_fault_[i]) {
            if (t.fault != 0) {
                RCLCPP_ERROR(this->get_logger(),
                    "Motor %d (%s) FAULT! Code: %d — commands blocked until STOP sent",
                    i + 1, JOINT_NAMES[i], t.fault);
                publishLog("# FAULT motor " + std::to_string(i + 1)
                    + " (" + JOINT_NAMES[i] + ") code=" + std::to_string(t.fault));
            } else if (last_fault_[i] != 0) {
                RCLCPP_INFO(this->get_logger(),
                    "Motor %d (%s) fault cleared", i + 1, JOINT_NAMES[i]);
                publishLog("# Motor " + std::to_string(i + 1)
                    + " (" + JOINT_NAMES[i] + ") fault cleared");
            }
            last_fault_[i] = t.fault;
        }

        // Mode state changed
        if (t.mode != last_mode_[i]) {
            // moteus Mode::kFault = 1, Mode::kStopped = 0
            if (t.mode == 1) {
                RCLCPP_ERROR(this->get_logger(),
                    "Motor %d (%s) entered FAULT mode", i + 1, JOINT_NAMES[i]);
            } else if (t.mode == 0 && last_mode_[i] != 0) {
                RCLCPP_WARN(this->get_logger(),
                    "Motor %d (%s) is STOPPED (driver disabled)", i + 1, JOINT_NAMES[i]);
            }
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

        auto& c = configs_[i];
        float padding = c.position_warn_rev_padding;
        bool near_max = t.position >= (c.position_max - padding);
        bool near_min = t.position <= (c.position_min + padding);

        if (near_max || near_min) {
            if (!position_alert_raised_[i]) {
                position_alert_raised_[i] = true;

                const char* which = near_max ? "MAX" : "MIN";
                float limit = near_max ? c.position_max : c.position_min;

                RCLCPP_WARN(this->get_logger(),
                    "Motor %d (%s) near %s position limit! "
                    "pos=%.3f limit=%.3f (padding=%.3f)",
                    i + 1, JOINT_NAMES[i], which,
                    t.position, limit, padding);

                publishLog("# WARNING motor " + std::to_string(i + 1)
                    + " (" + JOINT_NAMES[i] + ") near " + which
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
