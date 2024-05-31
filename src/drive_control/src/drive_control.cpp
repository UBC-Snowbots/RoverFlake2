#include <drive_control.h>

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriveControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}