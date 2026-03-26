#include "moteus_driver_node.h"
#include <cmath>

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

    config_log_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/arm/config_log", qos);

    command_sub_ = this->create_subscription<rover_msgs::msg::ArmCommand>(
        "/arm/command", qos,
        std::bind(&MoteusDriverNode::commandCallback, this, std::placeholders::_1));

    // Push configurations to motors at startup
    configureMotors();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MoteusDriverNode::poll, this));

    RCLCPP_INFO(this->get_logger(),
        "Moteus driver started: polling %d motors at 10 Hz", NUM_MOTORS);
}

void MoteusDriverNode::configureMotors() {
    static const char* JOINT_NAMES[] = {
        "Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "End Effector"
    };

    for (int i = 0; i < NUM_MOTORS; i++) {
        configureMotor(i + 1, *controllers_[i]);

        // Publish config summary to /arm/config_log for the HMI
        auto& c = configs_[i];
        std_msgs::msg::String log_msg;
        log_msg.data = std::string("# Motor ") + std::to_string(i + 1)
            + " (" + JOINT_NAMES[i] + ") configured:"
            + " kp=" + c.format(c.kp)
            + " kd=" + c.format(c.kd)
            + " max_current=" + c.format(c.max_current_A) + "A"
            + " pos=[" + c.format(c.position_min) + ", " + c.format(c.position_max) + "]rev"
            + " max_vel=" + c.format(c.max_velocity) + "rev/s"
            + " gear=" + c.format(c.gear_reduction);
        config_log_pub_->publish(log_msg);
    }
}

void MoteusDriverNode::configureMotor(int motor_id, mot::Controller& controller) {
    auto maybe_state = controller.SetQuery();
    if (!maybe_state) {
        RCLCPP_WARN(this->get_logger(),
            "Motor %d NOT CONNECTED. Skipping config.", motor_id);

        std_msgs::msg::String log_msg;
        log_msg.data = "# Motor " + std::to_string(motor_id) + " NOT CONNECTED — skipped";
        config_log_pub_->publish(log_msg);
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

void MoteusDriverNode::commandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);

    char cmd_type = msg->cmd_type;

    if (cmd_type == CMD_STOP) {
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

void MoteusDriverNode::poll() {
    std::array<MotorCommand, NUM_MOTORS> cmds{};
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmds = pending_cmds_;
        pending_cmds_ = {};
    }

    std::vector<mot::CanFdFrame> frames;
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (cmds[i].active && cmds[i].is_stop) {
            frames.push_back(controllers_[i]->MakeStop());
        } else if (cmds[i].active) {
            mot::PositionMode::Command pos_cmd;
            pos_cmd.position = cmds[i].position;
            pos_cmd.velocity = cmds[i].velocity;
            if (!std::isnan(cmds[i].max_torque))
                pos_cmd.maximum_torque = cmds[i].max_torque;
            frames.push_back(controllers_[i]->MakePosition(pos_cmd));
        } else {
            frames.push_back(controllers_[i]->MakeQuery());
        }
    }

    std::vector<mot::CanFdFrame> replies;
    transport_->BlockingCycle(frames.data(), frames.size(), &replies);

    rover_msgs::msg::MoteusArmStatus status_msg;
    status_msg.status.resize(NUM_MOTORS);
    status_msg.config.resize(NUM_MOTORS);

    // Fill config from our stored configs
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
    }

    for (const auto& frame : replies) {
        int id = frame.source;
        if (id < 1 || id > NUM_MOTORS) continue;

        auto r = mot::Query::Parse(frame.data, frame.size);
        auto& s = status_msg.status[id - 1];
        s.curr_position = r.position;
        s.curr_velocity = r.velocity;
        s.curr_torque = r.torque;
        s.curr_voltage_volts = r.voltage;
        s.driver_temp_degreesc = r.temperature;
        s.moteus_mode = static_cast<int16_t>(r.mode);
        s.moteus_fault = static_cast<int16_t>(r.fault);
    }

    feedback_pub_->publish(status_msg);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoteusDriverNode>());
    rclcpp::shutdown();
    return 0;
}
