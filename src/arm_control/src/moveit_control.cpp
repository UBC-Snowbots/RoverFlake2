//this is a sample node, with a joy input
#include "moveit_control.h"
#include <cmath>

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
      servo_msg->header.frame_id = "base_link";
      break;
    }
    case CARTESIAN_EE_FRAME:
        servo_msg->header.frame_id = "ee_base_link";

      break;

  }

      // Apply deadzone to filter joystick noise
      auto applyDeadzone = [](double val, double dz) -> double {
        return (std::abs(val) < dz) ? 0.0 : val;
      };
      const double DEADZONE = 0.15;

      // Log raw axes on every callback (throttled to 1Hz) so we can see rest values
      if (joy_msg->axes.size() >= 6) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "RAW JOY axes[0-5]: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] num_buttons=%zu btn[0]=%d btn[1]=%d",
          joy_msg->axes[0], joy_msg->axes[1], joy_msg->axes[2],
          joy_msg->axes[3], joy_msg->axes[4], joy_msg->axes[5],
          joy_msg->buttons.size(),
          (joy_msg->buttons.size() > 0) ? joy_msg->buttons[0] : -1,
          (joy_msg->buttons.size() > 1) ? joy_msg->buttons[1] : -1);
      }

      // Saitek Cyborg USB Stick mapping:
      // axes[0] = stick X (left/right)
      // axes[1] = stick Y (forward/back)
      // axes[2] = throttle slider (-1.0 to 1.0) -- DO NOT USE for continuous motion
      // axes[3] = stick twist/rudder (rotation)
      // axes[4] = hat X
      // axes[5] = hat Y
      // Use only stick axes for arm control:
      double lx = applyDeadzone(-joy_msg->axes[0], DEADZONE);   // stick X -> linear X
      double ly = applyDeadzone(joy_msg->axes[1], DEADZONE);    // stick Y -> linear Y
      double lz = 0.0;
      // Use hat Y (axes[5]) for linear Z (up/down) if available
      if (joy_msg->axes.size() > 5) {
        lz = applyDeadzone(joy_msg->axes[5], DEADZONE);
      }
      double ax = 0.0;  // angular X - use hat X if available
      if (joy_msg->axes.size() > 4) {
        ax = applyDeadzone(joy_msg->axes[4], DEADZONE);
      }
      double ay = 0.0;  // angular Y - use stick twist
      ay = applyDeadzone(joy_msg->axes[3], DEADZONE);
      double az = 0.0;  // not mapped (need more axes)

      servo_msg->twist.linear.x = lx;
      servo_msg->twist.linear.y = ly;
      servo_msg->twist.linear.z = lz;
      servo_msg->twist.angular.x = ax;
      servo_msg->twist.angular.y = ay;
      servo_msg->twist.angular.z = az;

      bool is_zero = (lx == 0.0 && ly == 0.0 && lz == 0.0 && ax == 0.0 && ay == 0.0 && az == 0.0);

      if (!is_zero) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "Twist cmd: lin[%.3f, %.3f, %.3f] ang[%.3f, %.3f, %.3f] frame=%s",
          lx, ly, lz, ax, ay, az, servo_msg->header.frame_id.c_str());
      }

      twist_cmd_publisher->publish(std::move(servo_msg));

      bool gripper_btn = (joy_msg->buttons.size() > GRIPPER_TOGGLE_BTN) && joy_msg->buttons[GRIPPER_TOGGLE_BTN];

      // Debug: log every button press/release change on button 1
      if (gripper_btn != prev_gripper_btn_) {
        RCLCPP_INFO(this->get_logger(), "GRIPPER BTN[%d] changed: %d -> %d (buttons.size()=%zu)",
          GRIPPER_TOGGLE_BTN, (int)prev_gripper_btn_, (int)gripper_btn, joy_msg->buttons.size());
      }

      if (gripper_btn && !prev_gripper_btn_) {
        gripper_open_ = !gripper_open_;
        double val = gripper_open_ ? GRIPPER_OPEN_VALUE : GRIPPER_CLOSE_VALUE;
        sendGripperCommand(val);
        RCLCPP_INFO(this->get_logger(), "Gripper %s command sent (end_effector=%.1f)",
          gripper_open_ ? "OPEN" : "CLOSE", val);
      }

      prev_gripper_btn_ = gripper_btn;
}

void ArmMoveitControl::sendGripperCommand(double value) {
  auto cmd = std::make_unique<rover_msgs::msg::ArmCommand>();
  cmd->cmd_type = 'G';  // Gripper command type
  cmd->end_effector = value;
  arm_publisher->publish(std::move(cmd));
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

RCLCPP_INFO(node->get_logger(), "=== SERVO STARTED ===");
RCLCPP_INFO(node->get_logger(), "Servo twist topic: %s", servo_parameters->cartesian_command_in_topic.c_str());
RCLCPP_INFO(node->get_logger(), "Servo joint topic: %s", servo_parameters->joint_command_in_topic.c_str());
RCLCPP_INFO(node->get_logger(), "Servo command out topic: %s", servo_parameters->command_out_topic.c_str());
RCLCPP_INFO(node->get_logger(), "Planning frame: %s", servo_parameters->planning_frame.c_str());
RCLCPP_INFO(node->get_logger(), "EE frame: %s", servo_parameters->ee_frame_name.c_str());
RCLCPP_INFO(node->get_logger(), "Move group: %s", servo_parameters->move_group_name.c_str());
RCLCPP_INFO(node->get_logger(), "Scale linear: %.4f, rotational: %.4f", servo_parameters->linear_scale, servo_parameters->rotational_scale);
RCLCPP_INFO(node->get_logger(), "Publish period: %.4f", servo_parameters->publish_period);
RCLCPP_INFO(node->get_logger(), "Command in type: %s", servo_parameters->command_in_type.c_str());
RCLCPP_INFO(node->get_logger(), "=== END SERVO CONFIG ===");

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