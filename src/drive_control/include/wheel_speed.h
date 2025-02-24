#ifndef WHEEL_SPEED_NODE_H
#define WHEEL_SPEED_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include <vector>
#include "rclcpp/qos.hpp"  // Include QoS header

class WheelSpeedNode : public rclcpp::Node {
public:
    WheelSpeedNode();
    ~WheelSpeedNode() = default;

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_wheel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    static constexpr double WHEEL_RADIUS_METERS = 0.3;  // Wheel radius
    static constexpr double TRACK_WIDTH_METERS = 0.6;   // Distance between left and right wheels
};

#endif // WHEEL_SPEED_NODE_H
