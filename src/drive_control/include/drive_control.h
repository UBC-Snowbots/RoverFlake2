/*
 * Created By: Caleb Reynard
 * Created On: February 13th, 2025
 * Description: Header file for the DriveControlNode
 *              Defines the ROS node that listens to joystick inputs
 *              and publishes velocity commands.
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "controller_config.h" // ArmControllerConfig::process_joy_input + controller mappings (arm_control pkg)

#define MAX_LINEAR_SPEED_MPS 1.0
#define MAX_ANGULAR_SPEED_MPS 1.0
#define TURBO_SPEED_MULTIPLIER 3

/**
 * @brief DriveControlNode handles joystick input and publishes velocity commands.
 */
class DriveControlNode : public rclcpp::Node {
public:
    DriveControlNode();

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);  // Callback function for joystick input

    // Which physical gamepad /joy comes from. PS4_JOY_LINUX matches the
    // previously hardcoded mapping (axes 1/3, button 0 turbo).
    ArmControllerConfig::GameController game_controller = ArmControllerConfig::GameController::PS4_DRIVE;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;  // Publisher for velocity commands
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;  // Subscription to joystick input
};
