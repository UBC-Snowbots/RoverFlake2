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

void RoverViz::update_camera_pose() {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "rviz_view_frame";

        // Set your desired position and orientation for the camera
        t.transform.translation.x = 1.0;  // Change these values to move the camera
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 1.0;

        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;

        // Broadcast the transform
        cam_tf_broadcaster_->sendTransform(t);
    }

// void MainHMINode::changeCard(std::string target_card){
//      middle_stack->set_visible_child(target_card.c_str());
//                 // m_stack.set_visible_child_name(page_name);
//             // rclcpp::delay(4);
//         // std::this_thread::sleep_for(std::chrono::seconds(1));
//         // middle_stack->set_visible_child("full_control_card");
// }