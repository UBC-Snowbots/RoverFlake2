// TODO: make a good comment
#include "ArmSerialInterface.h"

#define PI 3.14159

ArmSerial::ArmSerial() : Node("ArmSerialDriver") {
  //? new arm offsets. To change, check arm_control/include/armControlParams.h Should be synced with moveit params

  for (int i = 0; i < NUM_JOINTS; i++) {
    axes[i].zero_rad = ArmConstants::axis_zero_rads[i];
    axes[i].dir = ArmConstants::axis_dirs[i];
#ifdef PRINTOUT_AXIS_PARAMS
    RCLCPP_INFO(this->get_logger(), "Axis %i /// DIR[ %i ] /// OFFSET TO URDF's ZERO_RAD[ %f ] ", i + 1, axes[i].dir, axes[i].zero_rad);
#endif
  }

  // auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(); //Very hack way of only using "live" messages - iffy, and may still
  // operate off of one stale message. in the future we should use a time stamped message, and check the stamp time against current time to
  // make sure msg is not stale.
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(); //? AHHHH WHAT THE FUCK IS A QOS

  arm_position_publisher = this->create_publisher<rover_msgs::msg::ArmCommand>("/arm/feedback", qos);
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);
  double period = 1.0 / COMM_POLL_RATE;

  current_arm_status.positions.resize(NUM_JOINTS);

  command_subscriber = this->create_subscription<rover_msgs::msg::ArmCommand>(
      "/arm/command", 10, std::bind(&ArmSerial::CommandCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "joints %d", NUM_JOINTS);
  if (!SIMULATE) {
    for (int i = 1; i <= NUM_JOINTS; i++) {
      moteus::Controller::Options options;
      options.id = i;
      controllers[i] = std::make_shared<moteus::Controller>(options);
    }

    // Stop everything to clear faults.
    for (const auto &pair : controllers) {
      pair.second->SetStop();
    }

    // does send and recv every 10ms- same as arm firmware?
    period = 1 / 100;
    transport = moteus::Controller::MakeSingletonTransport({});
    timer_ = this->create_wall_timer(std::chrono::duration<double>(period), std::bind(&ArmSerial::serial_rx, this));
  }

  sleep(0.1);

  int flag = 1;
}

// Entry Point
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmSerial>();

  RCLCPP_INFO(node->get_logger(), "ArmSerial init");
  // std::thread arm_thread(ArmSerial::SerialRxThread, std::ref(node));

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
// Callbacks

void ArmSerial::CommandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg) {
  char type = msg->cmd_type;
  switch (type) {
  case HOME_CMD:
    if (!SIMULATE) {
      sendHomeCmd(msg->cmd_value);
    }
    break;
  case COMM_CMD:
    if (!SIMULATE) {
      sendCommCmd(msg->cmd_value);
    }
    break;
  case TEST_LIMITS_CMD:
    if (!SIMULATE) {
      send_test_limits_command();
    }
    break;
  case ABS_POS_CMD:
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

      joint_state_publisher_->publish(joint_states_);

    } else {
      send_position_command(target_positions);
    }
    break;
  case ABS_VEL_CMD:
    // double sim_target_velocities[NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS_NO_EE; i++) {
      target_velocities[i] = msg->velocities[i];
#ifdef DEBUG_MSGS
      RCLCPP_INFO(this->get_logger(), "J%i, %lf", i, msg->velocities[i]);
#endif // DEBUG_MSGS
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
      joint_state_publisher_->publish(joint_states_);

    } else {

      send_velocity_command(target_velocities); //!
    }
    break;
  default:
    break;
  }
}

// Member functions

float ArmSerial::degToRad(float deg) {
  float rad = deg * 3.14159 / 180; // 14159265359

  return (rad);
}

float ArmSerial::firmToMoveitOffsetPos(float deg, int i) {

  float rad = degToRad(deg);
  return ((rad * axes[i].dir) + (axes[i].zero_rad));
}

float ArmSerial::firmToMoveitOffsetVel(float deg, int i) {

  float rad = degToRad(deg);

  return ((rad * axes[i].dir));
}

void ArmSerial::parseLimitSwitchTest(std::string msg) {
  // TODO: can remove, no need for limit switches i think, use pos feedback?
  // also can set moteus position limits using
  // 2>servopos.position_min
  // 2>servopos.position_max

  // int axis = 0;
  // int value = 5;
  // if (sscanf(msg.c_str(), "Limit Switch %d is %d.", &axis, &value) == 2) {
  //   if (axis >= 1 && axis <= 6) {
  //     if (value == 1 || value == 0) {
  //       this->current_limit_switches[axis - 1];
  //       RCLCPP_INFO(this->get_logger(), "limit switches updated");
  //     }
  //   }
  //   RCLCPP_ERROR(this->get_logger(), "Arm limit switch parsing failed");
  // }
}

void ArmSerial::parseArmAngleUart(std::string msg) {
  // TODO: need to recalculate
  // ROS_INFO("Parsing Angle buffer: %s", msg.c_str());
  sensor_msgs::msg::JointState joint_states_;
  joint_states_.position.resize(NUM_JOINTS_NO_EE + 2);
  joint_states_.velocity.resize(NUM_JOINTS_NO_EE + 2);
  joint_states_.name.resize(NUM_JOINTS_NO_EE + 2);

  if (sscanf(msg.c_str(), "$my_angleP(%f, %f, %f, %f, %f, %f, %f)\n", &axes[0].curr_pos, &axes[1].curr_pos, &axes[2].curr_pos,
             &axes[3].curr_pos, &axes[4].curr_pos, &axes[5].curr_pos, &axes[6].curr_pos) == NUM_JOINTS) {
    // All axes angles are in axes[i].des_angle_pos
    RCLCPP_INFO(this->get_logger(), "Absolute Angle Position Echo Accepted:");
    for (int i = 0; i < NUM_JOINTS_NO_EE; i++) {
      current_arm_status.positions[i] = axes[i].curr_pos;
      joint_states_.name[i] = joint_names[i];
      joint_states_.position[i] = firmToMoveitOffsetPos(axes[i].curr_pos, i);
      // joint_states_.velocity[i] = firmToMoveitOffsetVel(current_velocity[i], i);
      joint_states_.velocity[i] = firmToMoveitOffsetVel(target_velocities[i], i); // TODO make based off of real velocity from firmware
      // joint_states_.position[i] = (prev_joint_states.position[i]) + (joint_states_.velocity[i] * (current_time - prev_time).seconds());
      // prev_joint_states.position[i] = joint_states_.position[i];
      // joint_states_.position[i] -= axes[i].zero_rad;
    }
    rclcpp::Time current_time(joint_states_.header.stamp);
    rclcpp::Time prev_time(prev_joint_states.header.stamp);
    joint_states_.name[6] = joint_names[6];
    joint_states_.name[7] = joint_names[7];
    joint_states_.position[6] = firmToMoveitOffsetPos(axes[EE_INDEX].curr_pos, EE_INDEX);
    joint_states_.position[7] = firmToMoveitOffsetPos(axes[EE_INDEX].curr_pos * -1, EE_INDEX);
    joint_states_.velocity[6] = 0;
    joint_states_.velocity[7] = 0;

    joint_states_.header.stamp = rclcpp::Clock().now();
    arm_position_publisher->publish(current_arm_status);
    joint_state_publisher_->publish(joint_states_);
    // RCLCPP_INFO(this->get_logger(), "Absolute Angle Position Echo Update Successfull");
    // fresh_rx_angle = true;

  } else {
    // Error handling: could not parse all 6 angles, or message is messed up.
    RCLCPP_ERROR(this->get_logger(), "Absolute Angle Position Echo Rejected, incorrect syntax");
    // fresh_rx_angle = false;

    return;
  }
}

void ArmSerial::sendMsg(std::string outMsg) {
  // TODO: prob remove, do send in specific funcs

  //  teensy.write(outMsg);
  //  teensy.flushOutput();
}

void ArmSerial::sendHomeCmd(int target_axis) {
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

void ArmSerial::sendCommCmd(int target_state) {
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

void ArmSerial::serial_rx() {

  if (send_angles) {
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
  return;

  // rclcpp::Rate loop_rate(50);
  std::string next_char = "";
  std::string buffer = "";
  int timeoutCounter = 0;
  // zephyrComm.teensy.flushInput();
  if (teensy.available() > 0) {
    // ROS_WARN("Reading");

    // timeoutCounter ++;
    // next_char = teensy.read();
    buffer = teensy.read(RX_UART_BUFF);
    RCLCPP_WARN(this->get_logger(), "%s", buffer.c_str());
    // if(next_char == "\n" || next_char == "\r" || next_char == "\0"){
    //     timeoutCounter = RX_UART_BUFF;
    // }
    if (buffer.size() > 0) {
      if (buffer.find("Arm Ready") != std::string::npos) {
        homed = true;
        homing = false;
        // fresh_rx_angle = true;
      } else if (buffer.find("my_angleP") != std::string::npos) {
        parseArmAngleUart(buffer);
      } else if (buffer.find("Limit Switch")) {
        parseLimitSwitchTest(buffer);
      }
    }
  }
}

void ArmSerial::send_position_command(float pos[NUM_JOINTS]) {
  std::vector<moteus::CanFdFrame> command_frames;
  // Define the desired position for each servo.
  // You would typically get this from your application's logic.
  std::map<int, double> target_positions;
  // Example target positions (in revolutions)
  target_positions[1] = 0.5;
  target_positions[2] = -0.25;
  // ... add other servo positions

  // Accumulate all of our command CAN frames.
  for (const auto &pair : controllers) {
    moteus::PositionMode::Command position_command;
    position_command.position = target_positions[pair.first]; // Set the target position
    position_command.velocity = 0.0;                          // Set target velocity if desired
    position_command.feedforward_torque = 0.0;                // Set feedforward torque if desired

    command_frames.push_back(pair.second->MakePosition(position_command));
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
  // char tx_msg[TX_UART_BUFF];
  //
  // sprintf(tx_msg, "$P(%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f)\n", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6]);
  //
  // sendMsg(tx_msg);
  // RCLCPP_INFO(this->get_logger(), "Positions Sent %s", tx_msg);
}

void ArmSerial::send_velocity_command(float vel[NUM_JOINTS]) {

  char tx_msg[TX_UART_BUFF];

  sprintf(tx_msg, "$V(%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f)\n", vel[0], vel[1], vel[2], vel[3], vel[4], vel[5], vel[6]);
  for (int i = 0; i < NUM_JOINTS; i++) {
    current_velocity[i] = vel[i];
  }

  sendMsg(tx_msg);
#ifdef DEBUG_MSGS
  RCLCPP_INFO(this->get_logger(), "Velocities Sent %s", tx_msg);
#endif // DEBUG_MSGS
}

void ArmSerial::send_test_limits_command() {
  char tx_msg[TX_UART_BUFF];
  sprintf(tx_msg, "$t()\n");
  sendMsg(tx_msg);
  RCLCPP_INFO(this->get_logger(), "Test limits Sent %s", tx_msg);
}
