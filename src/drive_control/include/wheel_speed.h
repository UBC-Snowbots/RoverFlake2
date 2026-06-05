#ifndef WHEEL_SPEED_NODE_H
#define WHEEL_SPEED_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rover_msgs/msg/drive_feedback.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include <vector>
#include "rclcpp/qos.hpp"

struct WheelVelocities {
  std::vector<double> left_wheel_velocities;
  std::vector<double> right_wheel_velocities; 
};

#define NUM_MOTORS 6

#define WHEEL_RADIUS_METERS 0.1
#define DRIVE_WIDTH_METERS 0.9
#define DRIVE_LENGTH_METERS 0.4

#define PIVOT_DEADZONE_THRESHOLD 0.5

class WheelSpeedNode : public rclcpp::Node {
public:
    WheelSpeedNode();
    ~WheelSpeedNode() = default;

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    WheelVelocities tankDrive(double linear, double angular);

    /**
     * When the rover is turning using tank steering, there is a dead zone for each side of the drive train when linear = -angular
     * With only half the wheels spinning, it is easy for the rover to get stuck
     * To prevent this, this function will set the non-moving side to a small velocity in the opposite direction to the turning side
     */
    WheelVelocities preventPivotDeadzone(WheelVelocities current_wheel_velocities);

    WheelVelocities ackermann(double linear, double angular);

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_wheel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<rover_msgs::msg::DriveFeedback>::SharedPtr drive_feedback_sub_;
};

#endif // WHEEL_SPEED_NODE_H
