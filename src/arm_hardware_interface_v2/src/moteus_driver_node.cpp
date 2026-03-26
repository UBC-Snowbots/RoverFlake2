#include "moteus_driver_node.h"
#include <cmath>

MoteusDriverNode::MoteusDriverNode() : Node("moteus_driver") {
    transport_ = mot::Controller::MakeSingletonTransport({});

    for (int id = 1; id <= NUM_MOTORS; id++) {
        mot::Controller::Options opts;
        opts.id = id;
        controllers_.push_back(std::make_shared<mot::Controller>(opts));
    }

    auto qos = rclcpp::QoS(1).reliable().durability_volatile();

    feedback_pub_ = this->create_publisher<rover_msgs::msg::MoteusArmStatus>(
        "/arm/moteus_feedback", qos);

    command_sub_ = this->create_subscription<rover_msgs::msg::ArmCommand>(
        "/arm/command", qos,
        std::bind(&MoteusDriverNode::commandCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MoteusDriverNode::poll, this));

    RCLCPP_INFO(this->get_logger(),
        "Moteus driver started: polling %d motors at 10 Hz, "
        "publishing /arm/moteus_feedback, subscribing /arm/command",
        NUM_MOTORS);
}

void MoteusDriverNode::commandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);

    char cmd_type = msg->cmd_type;

    if (cmd_type == CMD_STOP) {
        // Stop all motors
        for (int i = 0; i < NUM_MOTORS; i++) {
            pending_cmds_[i].active = true;
            pending_cmds_[i].is_stop = true;
        }
        RCLCPP_INFO(this->get_logger(), "Command: STOP ALL");
        return;
    }

    if (cmd_type == CMD_ABS_POS) {
        // Position command: positions[] and velocities[] arrays
        for (int i = 0; i < NUM_MOTORS; i++) {
            double pos = (i < (int)msg->positions.size()) ? msg->positions[i] : NAN;
            double vel = (i < (int)msg->velocities.size()) ? msg->velocities[i] : NAN;

            // NaN position AND NaN velocity = don't command this motor
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
        // Velocity command: velocities[] array, NaN = skip
        for (int i = 0; i < NUM_MOTORS; i++) {
            double vel = (i < (int)msg->velocities.size()) ? msg->velocities[i] : NAN;
            if (std::isnan(vel)) continue;

            pending_cmds_[i].active = true;
            // Velocity of 0 = stop
            if (vel == 0.0) {
                pending_cmds_[i].is_stop = true;
            } else {
                pending_cmds_[i].is_stop = false;
                pending_cmds_[i].position = NAN;  // no position target
                pending_cmds_[i].velocity = vel;
                pending_cmds_[i].max_torque = NAN;
            }
        }
        return;
    }

    RCLCPP_WARN(this->get_logger(), "Unknown cmd_type: '%c' (%d)", cmd_type, (int)cmd_type);
}

void MoteusDriverNode::poll() {
    // Grab pending commands
    std::array<MotorCommand, NUM_MOTORS> cmds{};
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmds = pending_cmds_;
        pending_cmds_ = {};  // Clear after consuming
    }

    // Build CAN frames
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

    // Execute CAN cycle
    std::vector<mot::CanFdFrame> replies;
    transport_->BlockingCycle(frames.data(), frames.size(), &replies);

    // Build and publish feedback
    rover_msgs::msg::MoteusArmStatus status_msg;

    for (const auto& frame : replies) {
        int id = frame.source;
        if (id < 1 || id > NUM_MOTORS) continue;

        auto r = mot::Query::Parse(frame.data, frame.size);

        rover_msgs::msg::BldcServoStatus s;
        s.curr_position = r.position;
        s.curr_velocity = r.velocity;
        s.curr_torque = r.torque;
        s.curr_voltage_volts = r.voltage;
        s.driver_temp_degreesc = r.temperature;
        s.moteus_mode = static_cast<int16_t>(r.mode);
        s.moteus_fault = static_cast<int16_t>(r.fault);

        // Pad status array up to the motor index
        while ((int)status_msg.status.size() < id)
            status_msg.status.push_back(rover_msgs::msg::BldcServoStatus());
        status_msg.status[id - 1] = s;
    }

    // Ensure we always have NUM_MOTORS entries
    while ((int)status_msg.status.size() < NUM_MOTORS)
        status_msg.status.push_back(rover_msgs::msg::BldcServoStatus());

    feedback_pub_->publish(status_msg);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoteusDriverNode>());
    rclcpp::shutdown();
    return 0;
}
