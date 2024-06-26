#include <RoverViz.h>
int main(int argc, char* argv[]){

    // int nullc = 0;
    // char **nullv = nullptr;
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<RoverViz>();
    
    // if(argv == "hi"){
    //     RCLCPP_INFO(node->get_logger(), "hi");
    // }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



void RoverViz::armFeedbackCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "meow arm");
    (void)msg;
}

// void MainHMINode::changeCard(std::string target_card){
//      middle_stack->set_visible_child(target_card.c_str());
//                 // m_stack.set_visible_child_name(page_name);
//             // rclcpp::delay(4);
//         // std::this_thread::sleep_for(std::chrono::seconds(1));
//         // middle_stack->set_visible_child("full_control_card");
// }