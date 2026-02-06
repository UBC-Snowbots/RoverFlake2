// TODO: make a good comment
#include "ArmSerialInterface.h"

#define PI 3.14159

// conf set servo.default_timeout_s nan
//  By default, when commanded over
//  CAN, there is a watchdog which requires commands to be sent at
//  least every 100ms or the controller will enter a latched fault
//  state, disable by using above cmd in tview

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

  RCLCPP_INFO(this->get_logger(), "moteus jointsff %d", NUM_JOINTS);
  if (!SIMULATE) {
    for (int i = 1; i <= NUM_JOINTS; i++) {
      moteus::Controller::Options options;
      options.id = i;
      controllers[i] = std::make_shared<moteus::Controller>(options);
    }

    // Stop everything to clear faults and configure
    for (const auto &pair : controllers) {
      pair.second->SetStop();
      RCLCPP_INFO(this->get_logger(), "moteuss jointsff %d", NUM_JOINTS);
      ConfigureMotor(pair.first, *pair.second);
      RCLCPP_INFO(this->get_logger(), "moteuaas jointsff %d", NUM_JOINTS);
    }

    // does send and recv every 10ms- same as arm firmware?
    period = 1 / 100;
    transport = moteus::Controller::MakeSingletonTransport({});
    timer_ = this->create_wall_timer(std::chrono::duration<double>(period), std::bind(&ArmSerial::serial_rx, this));
  }

  sleep(0.1);
}

// Entry Point
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmSerial>();

  RCLCPP_INFO(node->get_logger(), "ArmSerial init");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

// Callbacks
void ArmSerial::CommandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg) {
  char type = msg->cmd_type;
  RCLCPP_INFO(this->get_logger(), "dfsf init %c", type);
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
      RCLCPP_INFO(this->get_logger(), "velovc %f", msg->velocities[i]);
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
    // RCLCPP_INFO(this->get_logger(), "Absolute Angle Position Echo Accepted:");
    for (int i = 0; i < NUM_JOINTS_NO_EE; i++) {
      current_arm_status.positions[i] = axes[i].curr_pos;
      RCLCPP_INFO(this->get_logger(), "Curpos %d, %lf", i, current_arm_status.positions[i]);
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

    for (const auto &frame : replies) {
      // frame.source is the ID of the motor that replied
      int motor_id = frame.source;

      // Parse the raw CAN data into a readable result
      // moteus::Query::Parse(data, size) converts the raw bytes into position/velocity/faults
      auto result = moteus::Query::Parse(frame.data, frame.size);

      RCLCPP_INFO(this->get_logger(), "motor2ID: %d | Mode: %2d | Fault: %2d | Pos: %.3f | Vel: %.3f | Trq: %.3f | Volt: %.1f | Temp: %.1f",
                  motor_id, static_cast<int>(result.mode), static_cast<int>(result.fault), result.position, result.velocity, result.torque,
                  result.voltage, result.temperature);
      // Check if the motor is reporting a fault (anything other than 0 is an error)
      if (result.mode == moteus::Mode::kFault) {
        RCLCPP_ERROR(this->get_logger(), "Motor %d FAULT Detected! Code: %d", motor_id, result.fault);
      }

      if (result.mode == moteus::Mode::kStopped) {
        RCLCPP_WARN(this->get_logger(), "Motor %d is STOPPED (Driver Disabled)", motor_id);
      }
    }

    // Finally, print out our current query results.
    char buf[4096] = {};
    std::string status_line;

    // Note: The original code had a confusing 'status_line += buf;' here
    // without 'buf' having been initialized with status data yet.
    // I'll keep the logic that builds the servo status line.

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
      // Convert position from revolutions (moteus default) to degrees (360 * rev)
      // and store it in the axes structure based on the CAN ID (pair.first).
      axes[pair.first - 1].curr_pos = (360 * r.position);

      RCLCPP_INFO(this->get_logger(), "Gotpos %d, %lf", pair.first, axes[pair.first - 1].curr_pos);
      status_line += buf;
    }

    // --- NEW CODE: Construct the message for parseArmAngleUart ---
    // Ensure you have all 7 axes data before calling parseArmAngleUart.
    // Assuming axes[0] through axes[6] correspond to the 7 joints.

    char angle_msg_buffer[256];
    int result = ::snprintf(angle_msg_buffer, sizeof(angle_msg_buffer), "$my_angleP(%f, %f, %f, %f, %f, %f, %f)\n", axes[0].curr_pos,
                            axes[1].curr_pos, axes[2].curr_pos, axes[3].curr_pos, axes[4].curr_pos, axes[5].curr_pos, axes[6].curr_pos);

    // Check if the snprintf succeeded (result > 0 and result < sizeof(angle_msg_buffer))
    if (result > 0 && (size_t)result < sizeof(angle_msg_buffer)) {
      // Pass the constructed string to the parsing function
      parseArmAngleUart(std::string(angle_msg_buffer));
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to construct angle message for parsing.");
    }
    // --- END NEW CODE ---

    // ::printf("%s  \r", status_line.c_str());
    // ::fflush(::stdout);
  }
}

void ArmSerial::send_position_command(float pos[NUM_JOINTS]) {
  RCLCPP_INFO(this->get_logger(), "Test limits Sent");
  std::vector<moteus::CanFdFrame> command_frames;

  // --- START: Update to use 'pos' parameter ---
  std::map<int, double> target_positions;

  // 1. Map the input 'pos' array (index 0 to NUM_JOINTS-1) to CAN IDs.
  // Assuming CAN ID starts at 1 and corresponds to the index + 1.
  // Also assuming 'pos' contains positions in RADIANS, which need conversion to REVOLUTIONS.

  // Conversion factor: Revolutions = Radians / (2 * pi)
  const double RADIANS_TO_REVOLUTIONS = 1.0 / (2.0 * M_PI); // M_PI is typically available in <cmath>

  for (int i = 0; i < NUM_JOINTS; ++i) {
    // Moteus CAN IDs are 1-based, so use (i + 1) as the key.
    int can_id = i + 1;

    // Convert input position (assumed Radians) to Moteus's expected Revolutions.
    double position_in_rev = (double)pos[i] * RADIANS_TO_REVOLUTIONS;

    // Store the converted target position.
    target_positions[can_id] = position_in_rev;
  }
  // --- END: Update to use 'pos' parameter ---

  // Accumulate all of our command CAN frames.
  for (const auto &pair : controllers) {
    // pair.first is the CAN ID
    // pair.second is the moteus::Controller object

    moteus::PositionMode::Command position_command;

    // Use the converted position from the map.
    // We use .at() for safety, assuming every controller CAN ID has a corresponding target_position.
    position_command.position = target_positions.at(pair.first);

    // Assuming we want the motor to stop upon reaching the target position.
    position_command.velocity = 0.0;
    position_command.feedforward_torque = 0.0;

    command_frames.push_back(pair.second->MakePosition(position_command));
  }

  // Now send them in a single call to Transport::Cycle.
  std::vector<moteus::CanFdFrame> replies;
  transport->BlockingCycle(&command_frames[0], command_frames.size(), &replies);

  // Finally, print out our current query results.
  char buf[4096] = {};
  std::string status_line;

  status_line += buf; // This line seems vestigial, keeping for context if it's meant to be there.

  // We parse these into a map to both sort and de-duplicate them,
  // and persist data in the event that any are missing.
  for (const auto &frame : replies) {
    servo_data[frame.source] = moteus::Query::Parse(frame.data, frame.size);
  }

  // --- Prepare message for parseArmAngleUart ---
  char angle_msg_buffer[256];

  // This part requires all 7 joint positions to be available in the 'axes' array
  // after the query/reply cycle has updated 'axes[pair.first].curr_pos = (360 * r.position);'

  for (const auto &pair : servo_data) {
    const auto r = pair.second;
    ::snprintf(buf, sizeof(buf) - 1, "%2d %3d p/v/t=(%7.3f,%7.3f,%7.3f)  ", pair.first, static_cast<int>(r.mode), r.position, r.velocity,
               r.torque);
    // Position converted from revolutions to DEGREES (360 * rev) and stored for parsing later.
    axes[pair.first].curr_pos = (360 * r.position);

    status_line += buf;
  }

  // Construct the message for parseArmAngleUart using the *updated* positions (in degrees).
  // index 7 is wrong, use EE_INDEX
  int result = ::snprintf(angle_msg_buffer, sizeof(angle_msg_buffer), "$my_angleP(%f, %f, %f, %f, %f, %f, %f)\n", axes[1].curr_pos,
                          axes[2].curr_pos, axes[3].curr_pos, axes[4].curr_pos, axes[5].curr_pos, axes[6].curr_pos, axes[7].curr_pos);

  if (result > 0 && (size_t)result < sizeof(angle_msg_buffer)) {
    // Pass the constructed string to the parsing function
    parseArmAngleUart(std::string(angle_msg_buffer));
  } else {
    // Log an error if snprintf failed or buffer was too small
    // (You may need access to the logger for proper ROS2 logging here)
  }
  // --- End message preparation ---

  // ::printf("%s  \r", status_line.c_str());
  // ::fflush(::stdout);
}

// TODO: should send stop cmd after vel cmd so motor doesnt kepp spinning
void ArmSerial::send_velocity_command(float vel[NUM_JOINTS]) {
  std::vector<moteus::CanFdFrame> command_frames;

  std::map<int, double> target_positions;

  // convert deg/s to revolutions/s
  const double DEG_TO_REVOLUTIONS = 1.0 / 360.0;

  // Accumulate all of our command CAN frames.
  for (const auto &pair : controllers) {
    // pair.first is the CAN ID
    // pair.second is the moteus::Controller object

    moteus::PositionMode::Command position_command;

    position_command.position = std::numeric_limits<double>::quiet_NaN();

    // Assuming we want the motor to stop upon reaching the target position.
    RCLCPP_INFO(this->get_logger(), "veloc send %f", static_cast<double>(vel[(pair.first)]));
    position_command.velocity = vel[pair.first - 1] * DEG_TO_REVOLUTIONS;
    // position_command.velocity = vel[pair.first - 1];
    RCLCPP_INFO(this->get_logger(), "vel %f id: %d", position_command.velocity, pair.first);

    if (position_command.velocity == 0) {
      RCLCPP_INFO(this->get_logger(), "velocity stop %f", position_command.velocity);
      pair.second->SetStop();
    } else {
      command_frames.push_back(pair.second->MakePosition(position_command));
    }
  }

  // Now send them in a single call to Transport::Cycle.
  std::vector<moteus::CanFdFrame> replies;
  if (!command_frames.empty()) {
    transport->BlockingCycle(&command_frames[0], command_frames.size(), &replies);
  } else {
    return;
  }

  // 3. VERIFICATION LOGIC
  // A. Check we got the expected number of replies
  if (replies.size() != command_frames.size()) {
    RCLCPP_WARN(this->get_logger(), "Packet Loss! Sent %zu, Received %zu", command_frames.size(), replies.size());
  }

  // B. Match replies to motors and check for faults
  for (const auto &frame : replies) {
    // frame.source is the ID of the motor that replied
    int motor_id = frame.source;

    // Parse the raw CAN data into a readable result
    // moteus::Query::Parse(data, size) converts the raw bytes into position/velocity/faults
    auto result = moteus::Query::Parse(frame.data, frame.size);

    RCLCPP_INFO(this->get_logger(), "motorID: %d | Mode: %2d | Fault: %2d | Pos: %.3f | Vel: %.3f | Trq: %.3f | Volt: %.1f | Temp: %.1f",
                motor_id, static_cast<int>(result.mode), static_cast<int>(result.fault), result.position, result.velocity, result.torque,
                result.voltage, result.temperature);
    // Check if the motor is reporting a fault (anything other than 0 is an error)
    // if (result.mode == moteus::Mode::kFault) {
    //   RCLCPP_ERROR(this->get_logger(), "Motor %d FAULT Detected! Code: %d", motor_id, result.fault);
    // }
    //
    // if (result.mode == moteus::Mode::kStopped) {
    //   RCLCPP_WARN(this->get_logger(), "Motor %d is STOPPED (Driver Disabled)", motor_id);
    // }
  }
}

void ArmSerial::send_test_limits_command() {
  char tx_msg[TX_UART_BUFF];
  sprintf(tx_msg, "$t()\n");
  sendMsg(tx_msg);
  RCLCPP_INFO(this->get_logger(), "Test limits Sent %s", tx_msg);
}

void ArmSerial::ConfigureMotor(int axis_number, mjbots::moteus::Controller &controller) {
  auto maybe_state = controller.SetQuery();
  if (!maybe_state) {
    RCLCPP_WARN(this->get_logger(), "Motor %d NOT CONNECTED. Skipping config.", axis_number);
    return; // <--- Abort! Don't try to send config commands
  }
  RCLCPP_INFO(this->get_logger(), "Motor %d detected. Starting config...", axis_number);

  std::vector<std::pair<std::string, std::string>> settings;
  switch (axis_number) {
  case 1: {
    MotorConfigAxis1 config;
    settings = config.get_configs();
    break;
  }
  case 2: {
    MotorConfigAxis2 config;
    settings = config.get_configs();
    break;
  }
  case 3: {
    MotorConfigAxis3 config;
    settings = config.get_configs();
    break;
  }
  case 4: {
    MotorConfigAxis4 config;
    settings = config.get_configs();
    break;
  }
  case 5: {
    MotorConfigAxis5 config;
    settings = config.get_configs();
    break;
  }
  case 6: {
    MotorConfigAxis6 config;
    settings = config.get_configs();
    break;
  }
  }

  for (const auto &pair : settings) {
    std::string cmd = "conf set " + pair.first + " " + pair.second;

    // DiagnosticCommand returns a std::string containing the motor's text reply
    auto reply = controller.DiagnosticCommand(cmd);

    RCLCPP_INFO(this->get_logger(), "Set Reply: %s | Key: %s | Value: %s", reply.c_str(), pair.first.c_str(), pair.second.c_str());
  }

  // should not uncomment
  //  Write to flash so settings persist after power cycle
  //  auto reply = controller.DiagnosticCommand("conf write");
  //  std::cout << "Config Write: " << reply << std::endl;
}
