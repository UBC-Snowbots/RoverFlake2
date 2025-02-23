/*
 * Created By: Caleb Reynard
 * Created On: February 13th, 2025
 * Description: Header file for the DriveControlNode
 *              Defines the ROS node that listens to joystick inputs
 *              and publishes velocity commands.
 */

#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

/**
 * @brief DriveControlNode handles joystick input and publishes velocity commands.
 */
class DriveControlNode : public rclcpp::Node {
public:
    DriveControlNode();  // Constructor

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);  // Callback function for joystick input

    static constexpr double MAX_LINEAR_SPEED = 1.0;  // Maximum linear speed (m/s)

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;  // Publisher for velocity commands
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;  // Subscription to joystick input
};

#endif // DRIVE_CONTROL_H

