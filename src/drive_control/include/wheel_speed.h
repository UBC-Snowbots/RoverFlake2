#ifndef WHEEL_SPEED_NODE_H
#define WHEEL_SPEED_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include <vector>
#include "rclcpp/qos.hpp"  // Include QoS header

struct WheelVelocities {
  std::vector<double> left_wheel_velocities;
  std::vector<double> right_wheel_velocities; 
};

#define NUM_MOTORS 6

#define WHEEL_RADIUS_METERS 0.1
#define DRIVE_WIDTH_METERS 0.9
#define DRIVE_LENGTH_METERS 0.4

#define MOTOR_START_THRESHOLD 0.1
#define MOTOR_STOP_THRESHOLD 0.05

class WheelSpeedNode : public rclcpp::Node {
public:
    WheelSpeedNode();
    ~WheelSpeedNode() = default;

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    WheelVelocities tankDrive(double linear, double angular);
    WheelVelocities preventPivotDeadzone(WheelVelocities wheel_velocities);
    WheelVelocities ackermann(double linear, double angular);

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_wheel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

#endif // WHEEL_SPEED_NODE_H
