#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/arm_command.hpp"
#include "rover_msgs/msg/moteus_arm_status.hpp"
#include "rover_msgs/msg/bldc_servo_status.hpp"
#include "rover_msgs/msg/bldc_servo_config.hpp"

#include "moteus.h"

namespace mot = mjbots::moteus;

constexpr int NUM_MOTORS = 6;

// Command codes (matching arm_hardware_interface convention)
constexpr char CMD_ABS_POS = 'P';
constexpr char CMD_ABS_VEL = 'V';
constexpr char CMD_STOP    = 'S';

class MoteusDriverNode : public rclcpp::Node {
public:
    MoteusDriverNode();

private:
    void poll();
    void commandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg);

    std::shared_ptr<mot::Transport> transport_;
    std::vector<std::shared_ptr<mot::Controller>> controllers_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<rover_msgs::msg::MoteusArmStatus>::SharedPtr feedback_pub_;
    rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr command_sub_;

    // Pending per-motor commands from the latest /arm/command message
    struct MotorCommand {
        bool active = false;
        bool is_stop = false;
        double position = 0.0;
        double velocity = 0.0;
        double max_torque = NAN;
    };

    std::mutex cmd_mutex_;
    std::array<MotorCommand, NUM_MOTORS> pending_cmds_{};
};
