#include <cbsManagerNode.h>


bool CBSManagerNode::attachPort(std::string port, int baudrate, int id){
if(port.size() == 0){
    RCLCPP_ERROR(this->get_logger(), "Port Open Failure: No port name given. Aborting serial connection.");
    delete this;
    return 0;
}

}
int main(int argc, char* argv[]){
    //Can use executors here to make more nodes, maybe a node for each serial port
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CBSManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}