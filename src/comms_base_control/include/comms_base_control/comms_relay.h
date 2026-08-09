#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "rover_msgs/msg/servo_command.hpp"
#include <pigpiod_if2.h>

#include <unordered_map>

#define SERVO_RP_GPIO_PIN 17   // rack-and-pinion drive servo 2500-500
#define SERVO_CLAW_GPIO_PIN 27   // catch/release latch servo   1750-500

#define SERVO_RP_MIN_PWM 500.0f
#define SERVO_RP_MAX_PWM 2500.0f
#define SERVO_CLAW_MIN_PWM 500.0f
#define SERVO_CLAW_MAX_PWM 1750.0f

class ServoControlNode : public rclcpp::Node {
public:
    ServoControlNode();
    ~ServoControlNode();

private:
    // Calibrated travel of one servo, resolved from the command's servo selector.
    struct ServoSpec {
        const char* name;
        int gpio_pin;
        float min_pwm;
        float max_pwm;
    };

    // Returns false (and leaves spec untouched) if the selector is not a known servo.
    bool lookupServo(uint8_t servo, ServoSpec& spec) const;

    void setServoPWM(int gpio_pin, float PWM);

    /*  Motion is one-shot: the servo moves to the commanded position, holds for
     *  hold_ms, then returns to its minimum pulse width.
     *
     *  Go to max/min PWM (mode 0/1)
     *  ~ ros2 topic pub --once /servo_control_node/servo_command rover_msgs/msg/ServoCommand "{servo: 0, mode: 1}"
     *  Go to a user chosen PWM (mode 2), holding for 3 s
     *  ~ ros2 topic pub --once /servo_control_node/servo_command rover_msgs/msg/ServoCommand "{servo: 1, mode: 2, pwm_us: 1200, hold_ms: 3000}"
    */
    void handleServoCommand(const rover_msgs::msg::ServoCommand::SharedPtr msg);

    // Schedules the return to rest, replacing any hold already running for this
    // servo so a fresh command is not cut short by the previous one.
    void scheduleReturnToRest(const ServoSpec& spec, uint16_t hold_ms);

    rclcpp::Subscription<rover_msgs::msg::ServoCommand>::SharedPtr servo_command_sub_;

    // One pending return-to-rest timer per servo, keyed by GPIO pin.
    std::unordered_map<int, rclcpp::TimerBase::SharedPtr> return_timers_;

    // Hold applied when a command leaves hold_ms at 0.
    int default_hold_ms_ = 1000;

    // Handle for the connection to the pigpio daemon; negative means not connected.
    int pi_ = -1;
};
