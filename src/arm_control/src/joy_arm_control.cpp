//this is a sample node, with a joy input
#include "joy_arm_control.h"



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmJoy>();

    // Example: send a command with a steering angle of 0.5 rad and speed of 1.0 m/s

    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}