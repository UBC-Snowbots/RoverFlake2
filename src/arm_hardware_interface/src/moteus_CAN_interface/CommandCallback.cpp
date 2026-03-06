#include "ArmMoteusInterface.h"

// This is the callback for the moteus node and is called whenever a message is
// published on the /arm/command topic.  It parses the message and sends the
// appropriate command to the moteus controllers and/or the sim.

void ArmCAN::CommandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg) {
  char type = msg->cmd_type;
  RCLCPP_INFO(this->get_logger(), "dfsf init %c", type);
  
  switch (type) {
    case HOME_CMD: // need to implement for new arm
      if (!SIMULATE) { sendHomeCmd(msg->cmd_value); } 
      break;
    case COMM_CMD:
      if (!SIMULATE) { sendCommCmd(msg->cmd_value); }
      // RCLCPP_INFO(this->get_logger(), "limit switches updated");
      // RCLCPP_ERROR(this->get_logger(), "Arm limit switch parsing failed");
      break;
    case TEST_LIMITS_CMD:
      if (!SIMULATE) { send_test_limits_command(); }
      break;
    case ABS_POS_CMD: // for sim (allows teleporting lol)
      float target_positions[NUM_JOINTS];
      for (int i = 0; i < NUM_JOINTS; i++) {
        target_positions[i] = msg->positions[i];
      }
      if (SIMULATE) {
        sensor_msgs::msg::JointState joint_states_;
        joint_states_.position.resize(NUM_JOINTS);
        joint_states_.name.resize(NUM_JOINTS);
        for (int i = 0; i < NUM_JOINTS; i++) {
          joint_states_.name[i] = joint_names[i];
          joint_states_.position[i] = firmToMoveitOffsetPos(target_positions[i], i);
          joint_states_.velocity[i] = firmToMoveitOffsetVel(current_velocity[i], i);
        }
        joint_states_.header.stamp = rclcpp::Clock().now();

        joint_state_publisher_->publish(joint_states_); // sim! :)

      } else {
        send_position_command(target_positions); // non-sim :(
      }
      break;
    case ABS_VEL_CMD:
      // double sim_target_velocities[NUM_JOINTS];
      for (int i = 0; i < NUM_JOINTS_NO_EE; i++) {
        RCLCPP_INFO(this->get_logger(), "velovc %f", msg->velocities[i]);
        target_velocities[i] = msg->velocities[i];
        target_velocities[EE_INDEX] = msg->end_effector * EE_SPEED_SCALE;
        // current_velocity[i] = msg->velocities[i];
      }
      if (SIMULATE) {
        sensor_msgs::msg::JointState joint_states_;
        // auto curr_time = this->get_clock()->now();
        joint_states_.header.stamp = rclcpp::Clock().now();

        joint_states_.velocity.resize(NUM_JOINTS);
        joint_states_.position.resize(NUM_JOINTS);
        joint_states_.name.resize(NUM_JOINTS);
        for (int i = 0; i < NUM_JOINTS; i++) {
          joint_states_.name[i] = joint_names[i];
          joint_states_.velocity[i] = firmToMoveitOffsetVel(target_velocities[i], i);
          rclcpp::Time current_time(joint_states_.header.stamp);
          rclcpp::Time prev_time(prev_joint_states.header.stamp);
          joint_states_.position[i] = (prev_joint_states.position[i]) + (joint_states_.velocity[i] * (current_time - prev_time).seconds());
          prev_joint_states.position[i] = joint_states_.position[i];
          joint_states_.position[i] -= axes[i].zero_rad;
        }
        prev_joint_states.header.stamp = joint_states_.header.stamp;
        
        joint_state_publisher_->publish(joint_states_); // sim! :)

      } else {

        send_velocity_command(target_velocities); // non-sim :(
      }
      break;
    default:
      break;
  }
}

void ArmCAN::sendHomeCmd(int target_axis) {
  
  // TODO: use moteus pos feedback to get const home position values

  // send home request
  // std::string home_msg;
  // if (target_axis != HOME_ALL_ID) {
  //   home_msg = "$h(" + std::to_string(target_axis) + ")\n";
  // } else {
  //   home_msg = "$h(A)\n";
  // }
  //
  // sendMsg(home_msg);
}

void ArmCAN::sendCommCmd(int target_state) {
  // send communication request
  std::string msg;
  if (target_state) {
    // msg = "$SCP(1)\n";
    send_angles = true;
  } else {
    // msg = "$SCP(0)\n";
    send_angles = false;
  }

  // sendMsg(msg);
}

