#ifndef MOTOR_CONTROL_NODE_H
#define MOTOR_CONTROL_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"  // Include for Float64 message type
#include "phidget22.h"
#include <vector>
#include <algorithm>  // For std::clamp

class MotorControlNode : public rclcpp::Node {
public:
    MotorControlNode();  // Constructor
    ~MotorControlNode(); // Destructor

private:
    // Add subscriber declarations for left and right wheel velocity
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_wheel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_wheel_sub_;

    // Private methods for handling motor control and Phidget errors
    void handlePhidgetError(PhidgetReturnCode ret, const std::string& action, int i);  // Handle Phidget errors
    void run_motors(std::vector<int> selected_motors, float velocity);  // Run motors at a specified velocity
    void check_motor_positions();  // Check and log motor positions periodically

    // Callback functions for left and right wheel velocity subscriptions
    void leftWheelCallback(const std_msgs::msg::Float64::SharedPtr msg);  // Callback for left wheel velocity
    void rightWheelCallback(const std_msgs::msg::Float64::SharedPtr msg);  // Callback for right wheel velocity

    // Motor-related variables
    static const int NUM_MOTORS = 6;  // Number of motors
    PhidgetBLDCMotorHandle motors[NUM_MOTORS];  // Array of motors
    PhidgetReturnCode ret, errorCode;  // Return codes for Phidget functions
    const char* errorString;  // Error string for logging
    char errorDetail[100];  // Detailed error message
    size_t errorDetailLen = 100;  // Length of the error detail buffer
    rclcpp::TimerBase::SharedPtr timer_;  // Timer to check motor positions periodically
};

#endif // MOTOR_CONTROL_NODE_H

