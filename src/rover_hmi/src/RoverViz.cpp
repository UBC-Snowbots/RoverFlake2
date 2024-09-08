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
        // geometry_msgs::msg::TransformStamped t;

        // t.header.stamp = this->get_clock()->now();
        // t.header.frame_id = "map";
        // t.child_frame_id = "rviz_view_frame";

        // // Set your desired position and orientation for the camera
        // t.transform.translation.x = 1.0;  // Change these values to move the camera
        // t.transform.translation.y = 0.0;
        // t.transform.translation.z = 1.0;

        // t.transform.rotation.x = 0.0;
        // t.transform.rotation.y = 0.0;
        // t.transform.rotation.z = 0.0;
        // t.transform.rotation.w = 1.0;
    // Define the position for the label
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().x() = 0;  // Adjust the x position
    text_pose.translation().y() = 0;  // Adjust the y position
    text_pose.translation().z() = 1;  // Adjust the z position
    

    // Publish the text label

//    std::string labelmeow = "meow";
    base_link_visual_tool->publishText(text_pose, "ROVER", rvt::WHITE, rvt::XXLARGE, true);
    // Trigger the visual tools to actually display the text
    text_pose.translation().z() = 0.1;
    double angle = M_PI / 4.0; // 45 degrees
    Eigen::AngleAxisd rotation_vector(angle, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();
    text_pose.rotate(rotation_vector);
    // Eigen::Isometry3d cylindar_end_point = Eigen::Vector3d::Identity();

        // Broadcast the transform
        // cam_tf_broadcaster_->sendTransform(t);
        // base_link_visual_tool->
    left_front_visual_tool->publishCylinder(text_pose, rvt::GREEN, 0.5, 0.03, "left_front_cyl");
    left_middle_visual_tool->publishCylinder(text_pose, rvt::BLUE, 0.5, 0.03, "right_front_cyl");


        base_link_visual_tool->trigger();
        left_bogie_visual_tool->trigger();
        left_front_visual_tool->trigger();
        left_middle_visual_tool->trigger();
        right_bogie_visual_tool->trigger();
        right_front_visual_tool->trigger();
        right_middle_visual_tool->trigger(); 


        // base_link_visual_tool->trigger();
    }

// void MainHMINode::changeCard(std::string target_card){
//      middle_stack->set_visible_child(target_card.c_str());
//                 // m_stack.set_visible_child_name(page_name);
//             // rclcpp::delay(4);
//         // std::this_thread::sleep_for(std::chrono::seconds(1));
//         // middle_stack->set_visible_child("full_control_card");
// }