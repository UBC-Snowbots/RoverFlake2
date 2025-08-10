#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp> // ehh
#include <rover_msgs/msg/arm_command.hpp>
#include <moteus_msgs/msg/position_command.hpp>
#include <armControlParams.h> 

#include <array>
#include <vector>
#include <string>
#include <cmath>


class MoteusControlAdapter : public rclcpp::Node {
public:
  MoteusControlAdapter();
 

private:


void CommandCallback(const rover_msgs::msg::ArmCommand::SharedPtr rover_msg);


  // --- Members
  rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr command_subscriber;
  // rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Publisher<rover_msgs::msg::ArmCommand>::SharedPtr feedback_pub;

  // Placeholder publishers (uncomment when you know exact types):
  // rclcpp::Publisher<moteus_msgs::msg::MultiVelocityCommand>::SharedPtr moteus_pub_;
  std::vector<rclcpp::Publisher<moteus_msgs::msg::PositionCommand>::SharedPtr> moteus_pubs;

//   std::array<double, kJoints> offsets_deg_{};
  std::vector<std::string> joint_names_;
};