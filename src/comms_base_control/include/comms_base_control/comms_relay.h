#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rover_msgs/msg/servo_command.hpp"
#include <pigpiod_if2.h>

#include "servo.hpp"

#include <map>
#include <unordered_map>
#include <vector>

/*  Servos come from config/servos.yaml (name, GPIO pin, type); each type's
 *  calibration lives in config/servo_types/<type>.yaml.
 *
 *  Open / close a servo (holds the position):
 *  ~ ros2 service call /servo_control_node/claw/open std_srvs/srv/Trigger
 *  ~ ros2 service call /servo_control_node/claw/close std_srvs/srv/Trigger
 */
class ServoControlNode : public rclcpp::Node {
public:
    // Throws std::runtime_error if the servo configuration is invalid.
    ServoControlNode();
    ~ServoControlNode();

private:
    struct ServoEntry {
        Servo servo;
        std::string on_shutdown;  // min | max | open | close | release
    };

    // Builds servos_ from the servos.<name>.* parameters.
    void loadServos(const std::string& servo_types_dir);

    /*  Motion is one-shot: the servo moves to the commanded position, holds for
     *  hold_ms, then returns to rest (its close_us position).
     *
     *  Go to max/min PWM (mode 0/1)
     *  ~ ros2 topic pub --once /servo_control_node/servo_command rover_msgs/msg/ServoCommand "{servo: 0, mode: 1}"
     *  Go to a user chosen PWM (mode 2), holding for 3 s
     *  ~ ros2 topic pub --once /servo_control_node/servo_command rover_msgs/msg/ServoCommand "{servo: 1, mode: 2, pwm_us: 1200, hold_ms: 3000}"
    */
    void handleServoCommand(const rover_msgs::msg::ServoCommand::SharedPtr msg);

    void handleOpenClose(const std::string& name, bool open,
                         std_srvs::srv::Trigger::Response& response);

    // Schedules the return to rest, replacing any hold already running for this
    // servo so a fresh command is not cut short by the previous one.
    void scheduleReturnToRest(const std::string& name, uint16_t hold_ms);

    std::map<std::string, ServoEntry> servos_;

    rclcpp::Subscription<rover_msgs::msg::ServoCommand>::SharedPtr servo_command_sub_;
    std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> open_close_srvs_;

    // One pending return-to-rest timer per servo, keyed by servo name.
    std::unordered_map<std::string, rclcpp::TimerBase::SharedPtr> return_timers_;

    // Hold applied when a command leaves hold_ms at 0.
    int default_hold_ms_ = 1000;

    // Handle for the connection to the pigpio daemon; negative means not connected.
    int pi_ = -1;
};
