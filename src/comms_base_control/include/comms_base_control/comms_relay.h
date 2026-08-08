#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "rover_msgs/srv/set_servo_limit.hpp"
#include "rover_msgs/srv/set_servo_pwm.hpp"
#include <pigpiod_if2.h>

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
    // Calibrated travel of one servo, resolved from the service's servo selector.
    struct ServoSpec {
        const char* name;
        int gpio_pin;
        float min_pwm;
        float max_pwm;
    };

    // Returns false (and leaves spec untouched) if the selector is not a known servo.
    bool lookupServo(uint8_t servo, ServoSpec& spec) const;

    void setServoPWM(int gpio_pin, float PWM);

    void handleSetServoLimit(
        const std::shared_ptr<rover_msgs::srv::SetServoLimit::Request> request,
        std::shared_ptr<rover_msgs::srv::SetServoLimit::Response> response);
    void handleSetServoPwm(
        const std::shared_ptr<rover_msgs::srv::SetServoPwm::Request> request,
        std::shared_ptr<rover_msgs::srv::SetServoPwm::Response> response);
    /*  Go to max/min PWM (0/1)
     *  ~ ros2 service call /servo_control_node/set_servo_limit rover_msgs/srv/SetServoLimit "{servo: 0, limit: 1}"
     *  Call user chosen PWM
     *  ~ ros2 service call /servo_control_node/set_servo_pwm   rover_msgs/srv/SetServoPwm   "{servo: 1, pwm_us: 1200}"
    */
    rclcpp::Service<rover_msgs::srv::SetServoLimit>::SharedPtr set_servo_limit_srv_;
    rclcpp::Service<rover_msgs::srv::SetServoPwm>::SharedPtr set_servo_pwm_srv_;

    // Handle for the connection to the pigpio daemon; negative means not connected.
    int pi_ = -1;
};
