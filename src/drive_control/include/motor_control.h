#ifndef MOTOR_CONTROL_NODE_H
#define MOTOR_CONTROL_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"  // Correct include for Float64MultiArray message type
#include "phidget22.h"
#include "rclcpp/qos.hpp"  // Include QoS header
#include <vector>
#include <algorithm>  // For std::clamp

class MotorControlNode : public rclcpp::Node {
public:
    MotorControlNode();  // Constructor
    ~MotorControlNode(); // Destructor

private:
    // Subscriber declarations for left and right wheel velocities as Float64MultiArray
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr left_wheel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr right_wheel_sub_;

    // Phidget error handling function
    void handlePhidgetError(PhidgetReturnCode ret, const std::string& action, int motor_id);

    // Method to run motors at a specified velocity
    void runMotors(const std::vector<int>& selected_motors, float velocity);

    // Periodic motor position check
    void checkMotorPositions();

    // Callback functions for left and right wheel velocity subscriptions
    void leftWheelCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void rightWheelCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // Function to print the target velocity of each motor
    void printTargetVelocity();  // New function declaration

    // Constants and motor-related variables
    static const int NUM_MOTORS = 6;  // Number of motors
    PhidgetBLDCMotorHandle motors[NUM_MOTORS];  // Array of motors
    PhidgetReturnCode ret, errorCode;  // Return codes for Phidget functions
    const char* errorString;  // Error string for logging
    char errorDetail[100];  // Detailed error message
    size_t errorDetailLen = 100;  // Length of the error detail buffer

        // Failsafe timeout (ms)
    int failsafe_timeout_ms = 2000;

    // Timer to check motor positions periodically
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // MOTOR_CONTROL_NODE_H
