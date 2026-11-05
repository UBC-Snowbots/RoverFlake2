#include "moteus_node.h"

MoteusNode::MoteusNode(const std::string& node_name, const std::string& topic_prefix, std::vector<MotorConfig> configs, std::vector<std::string> motor_names) : Node(node_name) {
    configs_ = configs;
    motor_names_ = motor_names;
    bus_ = std::make_unique<MoteusBus>(configs_.size());
    motor_cmds_.resize(configs_.size());
    telem_.resize(configs_.size());

    auto qos     = rclcpp::QoS(5).reliable().durability_volatile();
    auto log_qos = rclcpp::QoS(LOG_QUEUE_SIZE_MSGS).reliable().durability_volatile();

    config_log_pub_  = this->create_publisher<std_msgs::msg::String>("/" + topic_prefix + "/config_log", log_qos);

    config_update_sub_ = this->create_subscription<rover_msgs::msg::MoteusConfigUpdate>(
        "/" + topic_prefix + "/config_update", qos,
        std::bind(&MoteusNode::configUpdateCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&MoteusNode::run, this));

    
    last_fault_.resize(configs_.size());
    last_mode_.resize(configs_.size());
    position_alert_raised_.resize(configs_.size()); 
}

void MoteusNode::configureMotors() {
    for (int i = 0; i < (int)configs_.size(); i++) {
        configureMotor(i + 1);

        auto& c = configs_[i];
        publishLog(std::string("# Motor ") + std::to_string(i + 1)
            + " (" + motor_names_[i] + ") configured:"
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
            motor_id, motor_names_[motor_id - 1].c_str());
        publishLog("# Motor " + std::to_string(motor_id)
            + " (" + motor_names_[motor_id - 1] + ") NOT CONNECTED — skipped");
        return;
    }

    RCLCPP_INFO(this->get_logger(),
        "Motor %d (%s) detected. Pushing config...",
        motor_id, motor_names_[motor_id - 1].c_str());

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
    if (calibrating_.load()) return;

    // Stage 1-5
    prepareCommands();


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

void MoteusNode::applyConfigToMemory(int idx, const std::string& reg, float val) {
    auto& c = configs_[idx];
    if      (reg == "servo.pid_position.kp")        c.kp              = val;
    else if (reg == "servo.pid_position.ki")        c.ki              = val;
    else if (reg == "servo.pid_position.kd")        c.kd              = val;
    else if (reg == "servo.max_current_A")          c.max_current_A   = val;
    else if (reg == "servo.max_velocity"
          || reg == "servo.default_velocity_limit") c.max_velocity    = val;
    else if (reg == "servo.default_accel_limit")    c.max_acceleration = val;
    else if (reg == "servopos.position_min")        c.position_min    = val;
    else if (reg == "servopos.position_max")        c.position_max    = val;
    else if (reg == "servo.max_voltage")            c.max_voltage     = val;
    else if (reg == "servo.max_power_W")            c.max_power_W     = val;
    else if (reg == "servo.default_timeout_s")      c.def_timeout     = val;
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
    "Motor %d (%s) position set to zero (d exact 0)", i + 1, motor_names_[i].c_str());
    publishLog("# Motor " + std::to_string(i + 1)
        + " (" + motor_names_[i] + ") zeroed");
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
    for (int i = 0; i < (int)configs_.size(); i++) {
        auto& t = telem_[i];
        if (!t.connected) continue;

        if (t.fault != last_fault_[i]) {
            if (t.fault != 0) {
                RCLCPP_ERROR(this->get_logger(),
                    "Motor %d (%s) FAULT! Code: %d — commands blocked until STOP sent",
                    i + 1, motor_names_[i].c_str(), t.fault);
                publishLog("# FAULT motor " + std::to_string(i + 1)
                    + " (" + motor_names_[i]
                    + ") code=" + std::to_string(t.fault));
            } else if (last_fault_[i] != 0) {
                RCLCPP_INFO(this->get_logger(),
                    "Motor %d (%s) fault cleared", i + 1, motor_names_[i].c_str());
                publishLog("# Motor " + std::to_string(i + 1)
                    + " (" + motor_names_[i] + ") fault cleared");
            }
            last_fault_[i] = t.fault;
        }

        if (t.mode != last_mode_[i]) {
            if (t.mode == 1)
                RCLCPP_ERROR(this->get_logger(),
                    "Motor %d (%s) entered FAULT mode", i + 1, motor_names_[i].c_str());
            else if (t.mode == 0 && last_mode_[i] != 0)
                RCLCPP_WARN(this->get_logger(),
                    "Motor %d (%s) is STOPPED (driver disabled)", i + 1, motor_names_[i].c_str());
            last_mode_[i] = t.mode;
        }
    }
}


// ---------------------------------------------------------------------------
// Position limit alerts — warns when approaching configured bounds
// ---------------------------------------------------------------------------

void MoteusNode::checkAlerts() {
    for (int i = 0; i < (int)configs_.size(); i++) {
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
                    i + 1, motor_names_[i].c_str(), which,
                    t.position, limit, padding);
                publishLog("# WARNING motor " + std::to_string(i + 1)
                    + " (" + motor_names_[i] + ") near " + which
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
    if (idx < 0 || idx >= (int)configs_.size()) {
        RCLCPP_WARN(this->get_logger(),
            "configUpdate: motor_id %d out of range (0..%d)", idx, (int)configs_.size() - 1);
        return;
    }
    if (msg->register_name.empty() || std::isnan(msg->value)) return;

    const std::string val_str = std::to_string(msg->value);
    const std::string cmd = "conf set " + msg->register_name + " " + val_str;

    RCLCPP_INFO(this->get_logger(),
        "configUpdate motor %d (%s): %s = %s",
        idx + 1, motor_names_[idx].c_str(),
        msg->register_name.c_str(), val_str.c_str());
    publishLog("# conf motor " + std::to_string(idx + 1)
        + " (" + motor_names_[idx] + "): "
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