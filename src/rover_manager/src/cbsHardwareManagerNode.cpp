#include <cbsHardwareManagerNode.h>

void CBSHardwareManagerNode::armPanelPoll(){
    ArmJoyPanel.pollRX();
}

void CBSHardwareManagerNode::slowPollCycle(){
    LeftPanel_A.pollRX();
}

int main(int argc, char* argv[]){
    //Can use executors here to make more nodes, maybe a node for each serial port
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CBSHardwareManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}