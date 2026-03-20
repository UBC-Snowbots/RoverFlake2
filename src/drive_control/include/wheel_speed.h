#ifndef WHEEL_SPEED_NODE_H
#define WHEEL_SPEED_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rover_msgs/msg/drive_feedback.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include <vector>
#include "rclcpp/qos.hpp"  // Include QoS header
#include "motor_control.h"

#define WHEEL_RADIUS_METERS 0.1 // Radius of the wheels
#define DRIVE_WIDTH_METERS 0.9 // Distance between the centres of the left and right wheels
#define DRIVE_LENGTH_METERS 0.4 // Distance between the centre of the middle wheel and the centre of the front/back wheel on the same side

#define MOTOR_START_THRESHOLD 0.06
#define MOTOR_STOP_THRESHOLD 0.02

struct WheelSpeeds {
  std::vector<double> left_wheel_speeds;
  std::vector<double> right_wheel_speeds; 
};

class WheelSpeedNode : public rclcpp::Node {
public:
    WheelSpeedNode();
    ~WheelSpeedNode() = default;

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void driveFeedbackCallback(const rover_msgs::msg::DriveFeedback::SharedPtr msg);

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_wheel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<rover_msgs::msg::DriveFeedback>::SharedPtr drive_feedback_sub_;

    // Functions for converting cmd_vel_ messages to wheel speeds, for different drive control behavior
    WheelSpeeds tankDrive(double linear, double angular); // Basic tank drive steering
    WheelSpeeds applyHysteresis(WheelSpeeds current_wheel_speeds, WheelSpeeds target_wheel_speeds); // Schmitt trigger/hysteresis
    WheelSpeeds ackermann(double linear, double angular); // Electronic Ackermann Steering

    WheelSpeeds current_wheel_speeds;
};

#endif // WHEEL_SPEED_NODE_H
