#include <cbsDisplayManagerNode.h>


void cbsDisplayManagerNode::moveWindow(std::string window_name, bool fullscreen){
    std::string command;
    
}


int main(int argc, char* argv[]){
    //Can use executors here to make more nodes, maybe a node for each serial port
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cbsDisplayManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}