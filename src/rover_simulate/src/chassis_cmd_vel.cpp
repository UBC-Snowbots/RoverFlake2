
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"



class GazeboChassisController : public rclcpp::Node
{
public:
  GazeboChassisController()
  : Node("sim_velocity_controller")
  {
    sim_motors_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 10);
    cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&GazeboChassisController::cmdVelCallback, this, std::placeholders::_1));
    // timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&GazeboChassisController::publish_velocity_command, this));
  }

  ~GazeboChassisController(){
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
    // publisher_->publish(simple_msg);
    // RCLCPP_INFO(this->get_logger(), "Publishing velocity command");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr sim_motors_pub;
  rclcpp::TimerBase::SharedPtr timer_;
};

void GazeboChassisController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
  RCLCPP_INFO(this->get_logger(), "Meow");
    std_msgs::msg::Float64MultiArray out_msg;
    float right_vel = msg->linear.x + msg->angular.z;
    float left_vel = -msg->linear.x + msg->angular.z;
    out_msg.data.resize(6);
    out_msg.data[0] = right_vel;
    out_msg.data[1] = right_vel;
    out_msg.data[2] = right_vel;
    out_msg.data[3] = left_vel;
    out_msg.data[4] = left_vel;
    out_msg.data[5] = left_vel;
    sim_motors_pub->publish(out_msg);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GazeboChassisController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

