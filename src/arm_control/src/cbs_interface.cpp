//* This node will take the cbs topic for the arm panel and send things around
#include "cbs_interface.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kPanelCenter = 50.0;
constexpr double kPanelRange = 50.0;
constexpr double kPanelDeadzone = 0.08;
constexpr double kIkLinearSpeed = 0.5;   // unitless, matches MoveIt Servo scale.linear
constexpr double kIkAngularSpeed = 0.6;  // unitless, matches joy_arm_control rotational range
constexpr double kEeCloseSpeed = -50.0;
constexpr double kEeOpenSpeed = 50.0;

double normalize_panel_axis(int raw_value) {
    const double normalized = (static_cast<double>(raw_value) - kPanelCenter) / kPanelRange;
    const double clamped = std::max(-1.0, std::min(1.0, normalized));
    return (std::abs(clamped) < kPanelDeadzone) ? 0.0 : clamped;
}

double end_effector_from_buttons(bool left_btn, bool right_btn) {
    if (left_btn && right_btn) {
        return 0.0;
    }
    if (left_btn) {
        return kEeCloseSpeed;
    }
    if (right_btn) {
        return kEeOpenSpeed;
    }
    return 0.0;
}

}  // namespace

void CBSArmInterface::arm_panel_callback(const rover_msgs::msg::ArmPanel::SharedPtr msg){
    const double left_x = normalize_panel_axis(msg->left.x);
    const double left_y = normalize_panel_axis(msg->left.y);
    const double left_z = normalize_panel_axis(msg->left.z);
    const double right_x = normalize_panel_axis(msg->right.x);
    const double right_y = normalize_panel_axis(msg->right.y);
    const double right_z = normalize_panel_axis(msg->right.z);
    const double ee_command = end_effector_from_buttons(msg->left.button, msg->right.button);

   if(ik){
        geometry_msgs::msg::TwistStamped ik_msg;

    ik_msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    ik_msg.header.frame_id = "base_link";
    ik_msg.twist.linear.x = left_x * kIkLinearSpeed;
    ik_msg.twist.linear.y = -left_y * kIkLinearSpeed;
    ik_msg.twist.linear.z = right_x * kIkLinearSpeed;
    ik_msg.twist.angular.x = right_z * kIkAngularSpeed;
    ik_msg.twist.angular.y = -right_y * kIkAngularSpeed;
    ik_msg.twist.angular.z = left_z * kIkAngularSpeed;
    arm_ik_pub->publish(ik_msg);

    // Keep gripper/end-effector controllable while in IK mode.
    rover_msgs::msg::ArmCommand ee_msg;
    ee_msg.cmd_type = 'V';
    ee_msg.velocities.assign(NUM_JOINTS_NO_EE, 0.0);
    ee_msg.end_effector = ee_command;
    arm_cmd_publisher->publish(ee_msg);
   }else{
    rover_msgs::msg::ArmCommand cmd_msg;
    cmd_msg.cmd_type = 'V';
    cmd_msg.velocities.resize(NUM_JOINTS);
    cmd_msg.velocities[0] = left_x * max_joysticks_output_speed_deg[0];
    cmd_msg.velocities[1] = -left_y * max_joysticks_output_speed_deg[1];
    cmd_msg.velocities[2] = -right_y * max_joysticks_output_speed_deg[2];
    cmd_msg.velocities[3] = left_z * max_joysticks_output_speed_deg[3];
    cmd_msg.velocities[4] = right_x * max_joysticks_output_speed_deg[4];
    cmd_msg.velocities[5] = right_z * max_joysticks_output_speed_deg[5];
    cmd_msg.end_effector = ee_command;

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
