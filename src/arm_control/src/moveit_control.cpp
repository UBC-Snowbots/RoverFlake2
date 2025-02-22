//this is a sample node, with a joy input
#include "moveit_control.h"

void ArmMoveitControl::publishCommands(){
// 	if (count_ < 100)
// {
//   auto msg = std::make_unique<control_msgs::msg::JointJog>();
//   msg->header.stamp = this->get_clock()->now();
//   msg->joint_names.push_back("joint_turntable");
//   msg->velocities.push_back(0.3);
//   joint_cmd_publisher->publish(std::move(msg));
//   ++count_;
// }  else
//   {
//     auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
//     msg->header.stamp = this->get_clock()->now();
//     msg->header.frame_id = "base_base_link";
//     msg->twist.linear.x = 0.3;
//     msg->twist.angular.z = 0.5;
//     twist_cmd_publisher->publish(std::move(msg));
    
//   }

}
void ArmMoveitControl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
  auto servo_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  servo_msg->header.stamp = this->get_clock()->now();

  switch (joyControlMode)  {
    case CARTESIAN_BASE_FRAME:
    {
      servo_msg->header.frame_id = "link_0";
      break;
    }
    case CARTESIAN_EE_FRAME:
        // ee_msg->header.frame_id = "link_ender";
        servo_msg->header.frame_id = "link_tt";

      break;

  }
      servo_msg->twist.linear.x = -joy_msg->axes[0];
      servo_msg->twist.linear.y = joy_msg->axes[1];
      servo_msg->twist.linear.z = static_cast<_Float64>(joy_msg->buttons[4] - joy_msg->buttons[5])/2;
      servo_msg->twist.angular.x = joy_msg->axes[3];
      servo_msg->twist.angular.y = (-joy_msg->axes[5] + joy_msg->axes[2])/2;
      servo_msg->twist.angular.z = joy_msg->axes[4];
      twist_cmd_publisher->publish(std::move(servo_msg));


}



int main(int argc, char *argv[]) {
  // ArmConstants instance;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmMoveitControl>();
rclcpp::sleep_for(std::chrono::seconds(4));
    // Example: send a command with a steering angle of 0.5 rad and speed of 1.0 m/s

    auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node);
if (!servo_parameters)
{
//RCLCPP_ERROR(LOGGER, "Failed to load the servo parameters");
  return EXIT_FAILURE;
}
 auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
    node, "robot_description", tf_buffer, "planning_scene_monitor");   
if (planning_scene_monitor->getPlanningScene())
{
  planning_scene_monitor->startStateMonitor(ArmConstants::joint_states_topic); //!not sure why ArmConstants struct doesn't work here.
  planning_scene_monitor->setPlanningScenePublishingFrequency(50);
  planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                       "/servo_node/publish_planning_scene");
  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->providePlanningSceneService();
}
else
{
  RCLCPP_ERROR(node->get_logger(), "Planning scene not configured");
  return EXIT_FAILURE;
}    
auto servo = std::make_unique<moveit_servo::Servo>(node, servo_parameters, planning_scene_monitor);
servo->start();



rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}

float ArmMoveitControl::radToDeg(float rad){
  float deg = (rad *180.0) / 3.14159; //265359

  return(deg);
}

float ArmMoveitControl::moveitToFirmwareOffset(float rad, int i){

float deg;

  deg = (rad - axes[i].zero_rad)*axes[i].dir; //neg feedback loop?
    
deg = radToDeg(deg);

return (deg);


}

float ArmMoveitControl::moveitVelocityToFirmwareOffset(float rad, int i){

float deg;
    
deg = radToDeg(rad) * axes[i].dir;

return (deg);


}