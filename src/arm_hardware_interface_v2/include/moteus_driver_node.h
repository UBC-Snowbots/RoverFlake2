#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <vector>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/arm_command.hpp"
#include "rover_msgs/msg/moteus_arm_status.hpp"
#include "rover_msgs/msg/bldc_servo_status.hpp"
#include "rover_msgs/msg/bldc_servo_config.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "moteus.h"
#include "motor_config.h"

namespace mot = mjbots::moteus;

// Command codes (matching arm_hardware_interface convention)
constexpr char CMD_ABS_POS = 'P';
constexpr char CMD_ABS_VEL = 'V';
constexpr char CMD_STOP    = 'S';

static const char* JOINT_NAMES[] = {
    "Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "End Effector"
};

class MoteusDriverNode : public rclcpp::Node {
public:
    MoteusDriverNode();

private:
    void configureMotors();
    void configureMotor(int motor_id, mot::Controller& controller);
    void poll();
    void commandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg);
    void checkAlerts();
    void checkFaults();
    void publishLog(const std::string& msg);

    std::shared_ptr<mot::Transport> transport_;
    std::vector<std::shared_ptr<mot::Controller>> controllers_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<rover_msgs::msg::MoteusArmStatus>::SharedPtr feedback_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
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
    std::array<MotorCommand, NUM_MOTORS> active_cmds_{};  // persists until stop/override

    // Per-motor telemetry (updated each poll)
    struct MotorTelem {
        float position = 0;
        float velocity = 0;
        float torque = 0;
        float voltage = 0;
        float temperature = 0;
        int mode = 0;
        int fault = 0;
        bool connected = false;
    };
    std::array<MotorTelem, NUM_MOTORS> telem_{};

    // Position limit alert state (prevents log spam)
    std::array<bool, NUM_MOTORS> position_alert_raised_{};

    // Fault state tracking (prevents log spam)
    std::array<int, NUM_MOTORS> last_fault_{};
    std::array<int, NUM_MOTORS> last_mode_{};
};
