#ifndef WHEEL_SPEED_NODE_H
#define WHEEL_SPEED_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include <vector>
#include "rclcpp/qos.hpp"  // Include QoS header

// Define the forward bias constant for when the speed is zero
static constexpr double FORWARD_BIAS = 0.05;  // Adjust this value for the desired bias (forward movement when speed = 0.0)

// Define the minimum speed for both forward and reverse motion
static constexpr double MIN_SPEED = 0.1;  // Minimum speed threshold to apply for forward or reverse movement

// WheelSpeedNode class definition
class WheelSpeedNode : public rclcpp::Node {
public:
    WheelSpeedNode();  // Constructor
    ~WheelSpeedNode() = default;  // Destructor

private:
    // Callback function to handle incoming velocity commands
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Publishers for left and right wheel speeds
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_wheel_pub_;
    
    // Subscription to receive velocity commands from the /cmd_vel topic
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // Helper function to apply deadzone removal and forward bias
    double applyDeadzoneAndBias(double speed);

    // Wheel constants
    static constexpr double WHEEL_RADIUS_METERS = 0.3;  // Wheel radius (in meters)
    static constexpr double TRACK_WIDTH_METERS = 0.6;   // Distance between left and right wheels (in meters)
};

#endif // WHEEL_SPEED_NODE_H
