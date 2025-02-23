#include <cbsHardwareManagerNode.h>

int CBSHardwareManagerNode::connectDevice(CBSDevice& dev){
    while(dev.findMyPort() != PORT_FOUND_SUCCESS && dev.AreYouSureImPluggedIn){
        dev.failed_connection_attempts++;
        if(dev.failed_connection_attempts > FAILED_CONNECTION_ATTEMPTS_MAX){
            dev.AreYouSureImPluggedIn = false;
            RCLCPP_WARN(this->get_logger(), "Are you sure %s %s %s is plugged in dawg?", bold(), dev.id.c_str(), reset());
            rclcpp::sleep_for(std::chrono::seconds(5));
            return TOO_MANY_FAILED_CONNECTION_ATTEMPTS;
        }
    }
    return DEVICE_CONNECITON_SUCCESS;
}


void CBSHardwareManagerNode::armPanelPoll(){
    ArmJoyPanel.pollRX();
}

void CBSHardwareManagerNode::slowPollCycle(){
    if(LeftPanel_A.is_connected){
        LeftPanel_A.pollRX();
    }
}

int main(int argc, char* argv[]){
    //Can use executors here to make more nodes, maybe a node for each serial port
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CBSHardwareManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}