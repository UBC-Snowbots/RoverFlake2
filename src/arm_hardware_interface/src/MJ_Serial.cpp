#include "ArmSerialInterface.h"

void ArmSerial::setupSerial() {
  for (int i = 1; i <= NUM_JOINTS; i++) {
    moteus::Controller::Options options;
    options.id = i;
    controllers[i] = std::make_shared<moteus::Controller>(options);
  }

  // Stop everything to clear faults.
  for (const auto &pair : controllers) {
    pair.second->SetStop();
  }

  transport = moteus::Controller::MakeSingletonTransport({});
  double period = 1.0 / COMM_POLL_RATE;
  timer_ = this->create_wall_timer(std::chrono::duration<double>(period), std::bind(&ArmSerial::serial_rx, this));
}

void ArmSerial::sendMsg(std::string outMsg) {
  // Send everything in outMsg through serial port
  // ROS_INFO("attempting send");
  //  std::string str_outMsg(reinterpret_cast<const char*>(outMsg), TX_UART_BUFF);
  // std::to_string(str_outMsg);  // +1 to copy the null-terminator
  teensy.write(outMsg);
  RCLCPP_ERROR(this->get_logger(), "Sent via serial: %s", outMsg.c_str());
  teensy.flushOutput();
}

void ArmSerial::parseLimitSwitchTest(std::string msg) {
  int axis = 0;
  int value = 5;
  if (sscanf(msg.c_str(), "Limit Switch %d is %d.", &axis, &value) == 2) {
    if (axis >= 1 && axis <= 6) {
      if (value == 1 || value == 0) {
        this->current_limit_switches[axis - 1];
        RCLCPP_INFO(this->get_logger(), "limit switches updated");
      }
    }
    RCLCPP_ERROR(this->get_logger(), "Arm limit switch parsing failed");
  }
}

void ArmSerial::parseArmAngleUart(std::string msg) {
  sensor_msgs::msg::JointState joint_states_;
  joint_states_.position.resize(NUM_JOINTS);
  joint_states_.velocity.resize(NUM_JOINTS);
  joint_states_.name.resize(NUM_JOINTS);
  for (int i = 0; i < NUM_JOINTS; i++) {
    current_arm_status.positions[i] = axes[i].curr_pos;
    joint_states_.name[i] = joint_names[i];
    joint_states_.position[i] = firmToMoveitOffsetPos(axes[i].curr_pos, i);
    joint_states_.velocity[i] = firmToMoveitOffsetVel(current_velocity[i], i);
  }

  joint_states_.header.stamp = rclcpp::Clock().now();
  arm_position_publisher->publish(current_arm_status);
  joint_state_publisher_->publish(joint_states_);
}

// A simple way to get the current time accurately as a double.
void ArmSerial::serial_rx() {
  std::vector<moteus::CanFdFrame> command_frames;

  // Accumulate all of our command CAN frames.
  for (const auto &pair : controllers) {
    command_frames.push_back(pair.second->MakeQuery());
  }

  // Now send them in a single call to Transport::Cycle.
  std::vector<moteus::CanFdFrame> replies;
  transport->BlockingCycle(&command_frames[0], command_frames.size(), &replies);

  // Finally, print out our current query results.

  char buf[4096] = {};
  std::string status_line;

  status_line += buf;

  char buf2[4096] = {};
  // We parse these into a map to both sort and de-duplicate them,
  // and persist data in the event that any are missing.
  for (const auto &frame : replies) {
    servo_data[frame.source] = moteus::Query::Parse(frame.data, frame.size);
  }

  for (const auto &pair : servo_data) {
    const auto r = pair.second;
    ::snprintf(buf, sizeof(buf) - 1, "%2d %3d p/v/t=(%7.3f,%7.3f,%7.3f)  ", pair.first, static_cast<int>(r.mode), r.position, r.velocity,
               r.torque);
    axes[pair.first].curr_pos = (360 * r.position);

    status_line += buf;
  }
  parseArmAngleUart("");

  ::printf("%s  \r", status_line.c_str());
  ::fflush(::stdout);
}

void ArmSerial::sendHomeCmd(int target_axis) {
  // send home request
  std::string home_msg;
  if (target_axis != HOME_ALL_ID) {
    home_msg = "$h(" + std::to_string(target_axis) + ")\n";
  } else {
    home_msg = "$h(A)\n";
  }

  sendMsg(home_msg);
}

void ArmSerial::sendCommCmd(int target_state) {
  // send communication request
  std::string msg;
  if (target_state) {
    msg = "$SCP(1)\n";
  } else {
    msg = "$SCP(0)\n";
  }

  sendMsg(msg);
}

void ArmSerial::send_position_command(float pos[NUM_JOINTS]) {

  char tx_msg[TX_UART_BUFF];

  sprintf(tx_msg, "$P(%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f)\n", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);

  sendMsg(tx_msg);
  RCLCPP_INFO(this->get_logger(), "Positions Sent %s", tx_msg);
}

void ArmSerial::send_velocity_command(float vel[NUM_JOINTS]) {

  char tx_msg[TX_UART_BUFF];

  sprintf(tx_msg, "$V(%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f)\n", vel[0], vel[1], vel[2], vel[3], vel[4], vel[5]);
  for (int i = 0; i < NUM_JOINTS; i++) {
    current_velocity[i] = vel[i];
  }

  sendMsg(tx_msg);
  RCLCPP_INFO(this->get_logger(), "Velocities Sent %s", tx_msg);
}

void ArmSerial::send_test_limits_command() {
  char tx_msg[TX_UART_BUFF];
  sprintf(tx_msg, "$t()\n");
  sendMsg(tx_msg);
  RCLCPP_INFO(this->get_logger(), "Test limits Sent %s", tx_msg);
}
