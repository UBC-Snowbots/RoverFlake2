#include "heart/heartNode.h"


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv); 
    rclcpp::NodeOptions options;
    // options.automatically_declare_parameters_from_overrides(true); // we don't have to declare params beforehand with this
    // options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<HeartNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
