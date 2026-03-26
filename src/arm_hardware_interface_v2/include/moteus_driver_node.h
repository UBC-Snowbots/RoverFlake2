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
#include "std_msgs/msg/string.hpp"

#include "moteus.h"
#include "motor_config.h"

namespace mot = mjbots::moteus;

// Command codes (matching arm_hardware_interface convention)
constexpr char CMD_ABS_POS = 'P';
constexpr char CMD_ABS_VEL = 'V';
constexpr char CMD_STOP    = 'S';

class MoteusDriverNode : public rclcpp::Node {
public:
    MoteusDriverNode();

private:
    void configureMotors();
    void configureMotor(int motor_id, mot::Controller& controller);
    void poll();
    void commandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg);

    std::shared_ptr<mot::Transport> transport_;
    std::vector<std::shared_ptr<mot::Controller>> controllers_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<rover_msgs::msg::MoteusArmStatus>::SharedPtr feedback_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr config_log_pub_;
    rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr command_sub_;

    std::vector<MotorConfig> configs_;

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
