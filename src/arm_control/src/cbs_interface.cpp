//* This node will take the cbs topic for the arm panel and send things around
#include "cbs_interface.h"
void CBSArmInterface::arm_panel_callback(const rover_msgs::msg::ArmPanel::SharedPtr msg){
    rover_msgs::msg::ArmCommand cmd_msg;
    cmd_msg.cmd_type = 'V'; //!SHOULD BE FROM ArmSerialProtocol.h
    cmd_msg.velocities.resize(NUM_JOINTS);
    cmd_msg.velocities[0] = (static_cast<float>(msg->left.x) - 50)/100 * max_speed_deg[0]*2;
     cmd_msg.velocities[1] = (static_cast<float>(msg->left.y) - 50)/100 * max_speed_deg[1]*2 *-1;
     cmd_msg.velocities[2] = (static_cast<float>(msg->right.y) - 50)/100 * max_speed_deg[2]*2*-1;
     cmd_msg.velocities[3] = (static_cast<float>(msg->left.z) - 50)/100 * max_speed_deg[3]*2;
     cmd_msg.velocities[4] = (static_cast<float>(msg->right.x) - 50)/100 * max_speed_deg[4]*2;
     cmd_msg.velocities[5] = (static_cast<float>(msg->right.z) - 50)/100 * max_speed_deg[5]*2;
    
    if(msg->left.button && msg->right.button){

    cmd_msg.end_effector = 0;
    }else{
    if(msg->left.button){

    cmd_msg.end_effector = -0.07;
    }
    if(msg->right.button){

    cmd_msg.end_effector = 0.07;
    }
    }

    arm_cmd_publisher->publish(cmd_msg);
}




int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CBSArmInterface>();

    // Example: send a command with a steering angle of 0.5 rad and speed of 1.0 m/s

    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}