
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"





class ChassisSimMotorController : public rclcpp::Node
{
public:
  ChassisSimMotorController()
  : Node("velocity_controller")
  {
    six_motor_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/velocity_controller/commands", 10);
    cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, 
            std::bind(&ChassisSimMotorController::cmdVelCallback, this, std::placeholders::_1));
    // timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ChassisVelocityController::publish_velocity_command, this));
  }

  ~ChassisSimMotorController(){
    RCLCPP_INFO(this->get_logger(), "Shutting down chassis' sim velocity controller");
  }


  void publish_velocity_command()
  {

    // std_msgs::msg::Float64MultiArray  simple_msg;
    // //? Index of each joint command
    // // joint_names = {
    // // 0  "back_left_wheel_joint",
    // // 1  "left_mid_wheel_joint",
    // // 2  "left_front_wheel_joint",
    // // 3  "right_front_wheel_joint",
    // // 4  "right_mid_wheel_joint",
    // // 5  "back_right_wheel_joint"
    // // };
    // simple_msg.data.resize(6);
    // for (int i = 0; i < 6; i++){
    // simple_msg.data[i] = 3; //spin 
    // }

    // // trajectory_msgs::msg::JointTrajectoryPoint point;
    // // point.velocities = {60.5, 60.5, 60.5, -60.5, -60.5, -60}; // Set the desired velocities for each joint
    // // point.positions = {10.0, 10.0, 10.0, -10.0, -10.0, -10.0}; // Set the desired velocities for each joint
    // // point.effort = {10.0, 10.0, 10.0, -10.0, -10.0, -10.0}; // Set the desired velocities for each joint
    // // point.time_from_start = rclcpp::Duration::from_seconds(1);

    // // trajectory.points.push_back(point);
    // publisher_->publish(simple_msg);
    // RCLCPP_INFO(this->get_logger(), "Publishing velocity command");
  }
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);


private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr six_motor_publisher;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
  rclcpp::TimerBase::SharedPtr timer_;
};


void ChassisSimMotorController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
    float lin = msg->linear.x;
    float ang = msg->angular.z;
    std_msgs::msg::Float64MultiArray six_motor_msg;

    //create a new message. left motors are 0,1,2  right motors are 3,4,5
    for(int i = 0; i < 6; i++){
        if(i < 3){
            six_motor_msg.data[i] = lin + ang;
        } else{
            six_motor_msg.data[i] = lin - ang;
        }

    }
    six_motor_publisher->publish(six_motor_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChassisSimMotorController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

