//* This node will take the cbs topic for the arm panel and send things around
#include "cbs_interface.h"
void CBSArmInterface::arm_panel_callback(const rover_msgs::msg::ArmPanel::SharedPtr msg){
   if(ik){
        geometry_msgs::msg::TwistStamped ik_msg;

    ik_msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    ik_msg.header.frame_id = "link_tt";
   //  ik_msg.twist.linear.x = static_cast<float>(msg->
    ik_msg.twist.linear.x = (static_cast<float>(msg->left.x) - 50)/100 * max_joysticks_output_speed_deg[0]*2 / 180;
     ik_msg.twist.linear.y = (static_cast<float>(msg->left.y) - 50)/100 * max_joysticks_output_speed_deg[1]*2 *-1 / 180;
     ik_msg.twist.linear.z = (static_cast<float>(msg->right.x) - 50)/100 * max_joysticks_output_speed_deg[4]*2 / 180;
     ik_msg.twist.angular.x = (static_cast<float>(msg->right.z) - 50)/100 * max_joysticks_output_speed_deg[5]*2 / 180;
    ik_msg.twist.angular.y = (static_cast<float>(msg->right.y) - 50)/100 * max_joysticks_output_speed_deg[2]*2*-1 / 180;
     ik_msg.twist.angular.z = (static_cast<float>(msg->left.z) - 50)/100 * max_joysticks_output_speed_deg[3]*2 / 180;
    // float value = ik_hmi_speed[index]; ///TODO get speed based on spinbuttons
    // switch (index)
    // {
    //     case 0: // Linear X
    //         ik_msg.twist.linear.x = value;
    //         break;
    //     case 1: // Linear Y
    //         ik_msg.twist.linear.y = value;
    //         break;
    //     case 2: // Linear Z
    //         ik_msg.twist.linear.z = value;
    //         break;
    //     case 3: // Angular X
    //         ik_msg.twist.angular.x = value;
    //         break;
    //     case 4: // Angular Y
    //         ik_msg.twist.angular.y = value;
    //         break;
    //     case 5: // Angular Z
    //         ik_msg.twist.angular.z = value;
    //         break;
    //     default:
    //         RCLCPP_WARN(this->get_logger(), "Invalid index: %d. Must be 0-5.", index);
    //         return;
    // }
    
    arm_ik_pub->publish(ik_msg);
   }else{

   

    rover_msgs::msg::ArmCommand cmd_msg;
    cmd_msg.cmd_type = 'V'; //!SHOULD BE FROM ArmSerialProtocol.h
    cmd_msg.velocities.resize(NUM_JOINTS);
    cmd_msg.velocities[0] = (static_cast<float>(msg->left.x) - 50)/100 * max_joysticks_output_speed_deg[0]*2;
     cmd_msg.velocities[1] = (static_cast<float>(msg->left.y) - 50)/100 * max_joysticks_output_speed_deg[1]*2 *-1;
     cmd_msg.velocities[2] = (static_cast<float>(msg->right.y) - 50)/100 * max_joysticks_output_speed_deg[2]*2*-1;
     cmd_msg.velocities[3] = (static_cast<float>(msg->left.z) - 50)/100 * max_joysticks_output_speed_deg[3]*2;
     cmd_msg.velocities[4] = (static_cast<float>(msg->right.x) - 50)/100 * max_joysticks_output_speed_deg[4]*2;
     cmd_msg.velocities[5] = (static_cast<float>(msg->right.z) - 50)/100 * max_joysticks_output_speed_deg[5]*2;
    
    if(msg->left.button && msg->right.button){

    cmd_msg.end_effector = 0;
    }else{
    if(msg->left.button){

    cmd_msg.end_effector = -50;//-0.07;
    }
    if(msg->right.button){

    cmd_msg.end_effector = 50;//0.07;
    }
    }

    arm_cmd_publisher->publish(cmd_msg);
}
}

void CBSArmInterface::left_panel_callback(const rover_msgs::msg::GenericPanel::SharedPtr msg){
    // rover_msgs::msg:: cmd_msg;
 if(msg->switches[0] == 0){
    ik = false;
 }else{
    ik = true;
 }
}




int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CBSArmInterface>();

    // Example: send a command with a steering angle of 0.5 rad and speed of 1.0 m/s

    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}