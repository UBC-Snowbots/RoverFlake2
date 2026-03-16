#include "ArmMoteusInterface.h"

// main :)
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmCAN>();

  RCLCPP_INFO(node->get_logger(), "ArmCAN init");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}