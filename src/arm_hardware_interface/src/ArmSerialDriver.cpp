//TODO: make a good comment
#include "ArmSerialDriver.h"


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmSerial>();

    RCLCPP_INFO(node->get_logger(), "ArmSerial init");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}