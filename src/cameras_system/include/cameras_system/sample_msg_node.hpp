#ifndef SAMPLE_MSG_NODE_HPP_
#define SAMPLE_MSG_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MsgNode : public rclcpp::Node
{
public:
  MsgNode();
private:
  void timer_callback();

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr msg_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // SAMPLE_MSG_NODE_HPP_
