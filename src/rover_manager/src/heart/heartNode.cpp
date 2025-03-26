#include "heart/heartNode.h"


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<HeartNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
