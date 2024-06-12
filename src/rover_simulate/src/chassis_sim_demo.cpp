
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"



class ChassisVelocityController : public rclcpp::Node
{
public:
  ChassisVelocityController()
  : Node("velocity_controller")
  {
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ChassisVelocityController::publish_velocity_command, this));
  }

  ~ChassisVelocityController(){
    RCLCPP_INFO(this->get_logger(), "Shutting down velocity controller");
  }


  void publish_velocity_command()
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = {
      "back_left_wheel_joint",
      "left_mid_wheel_joint",
      "left_front_wheel_joint",
      "right_front_wheel_joint",
      "right_mid_wheel_joint",
      "back_right_wheel_joint"
    };

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.velocities = {60.5, 60.5, 60.5, -60.5, -60.5, -60}; // Set the desired velocities for each joint
    point.positions = {10.0, 10.0, 10.0, -10.0, -10.0, -10.0}; // Set the desired velocities for each joint
    point.effort = {10.0, 10.0, 10.0, -10.0, -10.0, -10.0}; // Set the desired velocities for each joint
    point.time_from_start = rclcpp::Duration::from_seconds(1);

    trajectory.points.push_back(point);
    publisher_->publish(trajectory);
    RCLCPP_INFO(this->get_logger(), "Publishing velocity command");
  }

private:
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChassisVelocityController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

