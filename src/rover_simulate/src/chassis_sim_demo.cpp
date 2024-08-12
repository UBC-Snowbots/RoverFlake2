
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"



class ChassisVelocityController : public rclcpp::Node
{
public:
  ChassisVelocityController()
  : Node("velocity_controller")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ChassisVelocityController::publish_velocity_command, this));
  }

  ~ChassisVelocityController(){
    RCLCPP_INFO(this->get_logger(), "Shutting down velocity controller");
  }


  void publish_velocity_command()
  {

    std_msgs::msg::Float64MultiArray  simple_msg;
    //? Index of each joint command
    // joint_names = {
    // 0  "back_left_wheel_joint",
    // 1  "left_mid_wheel_joint",
    // 2  "left_front_wheel_joint",
    // 3  "right_front_wheel_joint",
    // 4  "right_mid_wheel_joint",
    // 5  "back_right_wheel_joint"
    // };
    simple_msg.data.resize(6);
    for (int i = 0; i < 6; i++){
    simple_msg.data[i] = 3; //spin 
    }

    // trajectory_msgs::msg::JointTrajectoryPoint point;
    // point.velocities = {60.5, 60.5, 60.5, -60.5, -60.5, -60}; // Set the desired velocities for each joint
    // point.positions = {10.0, 10.0, 10.0, -10.0, -10.0, -10.0}; // Set the desired velocities for each joint
    // point.effort = {10.0, 10.0, 10.0, -10.0, -10.0, -10.0}; // Set the desired velocities for each joint
    // point.time_from_start = rclcpp::Duration::from_seconds(1);

    // trajectory.points.push_back(point);
    publisher_->publish(simple_msg);
    RCLCPP_INFO(this->get_logger(), "Publishing velocity command");
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  // rclcpp::init(argc, argv);
  // auto node = std::make_shared<ChassisVelocityController>();
  // rclcpp::spin(node);
  // rclcpp::shutdown();
  return 0;
}

