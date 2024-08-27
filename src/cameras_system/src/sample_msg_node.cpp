#include "cameras_system/sample_msg_node.hpp"


MsgNode::MsgNode()
: Node("sample_msg_node") {
  // Initialize Publisher
  msg_pub_  = this->create_publisher<std_msgs::msg::String>("byte_msg", 10);

  // Basic timer implementation
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&MsgNode::timer_callback, this));
}

void MsgNode::timer_callback() {
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = "111"; // Example in string format
    // RCLCPP_INFO(this->get_logger(), "String msg: %s", msg->data.c_str());
    msg_pub_->publish(*msg);

}

// // Subject to change according to msg type
// void CamerasSysNode::topic_callback(const std_msgs::msg::String::SharedPtr msg) {
//   RCLCPP_INFO(this->get_logger(), "Received msg: '%s'", msg->data.c_str());
// }


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MsgNode>());
  rclcpp::shutdown();
  return 0;
}
