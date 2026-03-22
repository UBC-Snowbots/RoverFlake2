#include "ArmMoteusInterface.h"

float ArmCAN::radToRev(float rad) {
  float rev = rad / (2.0 * M_PI);
  return (rev);
}

// new send_position_command that uses Moteus over parsing hacks
void ArmCAN::send_position_command(float pos_rad[NUM_JOINTS]) {
  std::vector<moteus::CanFdFrame> command_frames;

  for (const auto &[id, controller] : controllers) {
    const int idx = id - 1;

    moteus::PositionMode::Command cmd;
    cmd.position = radToRev(pos_rad[idx]);
    cmd.velocity = 0.0;  // stop at target

    motor_telem[idx].des_position = pos_rad[idx];
    command_frames.push_back(controller->MakePosition(cmd));
  }

  if (command_frames.empty()) return;

  std::vector<moteus::CanFdFrame> replies;
  transport->BlockingCycle(command_frames.data(), command_frames.size(), &replies);

  if (replies.size() != command_frames.size()) {
    RCLCPP_WARN(this->get_logger(), "Position cmd: sent %zu, got %zu replies",
                command_frames.size(), replies.size());
  }

  for (const auto &frame : replies) {
    auto r = moteus::Query::Parse(frame.data, frame.size);
    const int idx = frame.source - 1;
    if (idx >= 0 && idx < NUM_JOINTS) {
      axes[idx].curr_pos = r.position * 360.0;  // rev → deg
      motor_telem[idx].curr_position = r.position;
      motor_telem[idx].curr_velocity = r.velocity;
      motor_telem[idx].curr_torque_Nm = r.torque;
    }
    servo_data[frame.source] = r;
  }
}

// Notes: Very novel send_position_command (lots of parsing logic). 
// LEGACY: Move all motors to an absolute positionm (in rev -- takes in rad)
// void ArmCAN::send_position_command_legacy(float pos[NUM_JOINTS]) {
//   RCLCPP_INFO(this->get_logger(), "Test limits Sent");
//   std::vector<moteus::CanFdFrame> command_frames;
//   std::map<int, double> target_positions;
//   const double RADIANS_TO_REVOLUTIONS = 1.0 / (2.0 * M_PI); // convert from rad to rev

//   // store in CAN ID map target_positions
//   for (int i = 0; i < NUM_JOINTS; ++i) {
//     int can_id = i + 1;
//     double position_in_rev = (double)pos[i] * RADIANS_TO_REVOLUTIONS;

//     target_positions[can_id] = position_in_rev;
//   }

//   for (const auto &pair : controllers) {
//     moteus::PositionMode::Command position_command;
//     position_command.position = target_positions.at(pair.first);
//     position_command.velocity = 0.0;
//     position_command.feedforward_torque = 0.0;

//     // serialize into CAN-FD frame and add to command_frames
//     command_frames.push_back(pair.second->MakePosition(position_command));
//   }

//   std::vector<moteus::CanFdFrame> replies;
//   if (command_frames.empty()) { return; }
//   transport->BlockingCycle(&command_frames[0], command_frames.size(), &replies);

//   char buf[4096] = {};
//   std::string status_line;
//   status_line += buf;
//   for (const auto &frame : replies) {
//     servo_data[frame.source] = moteus::Query::Parse(frame.data, frame.size);
//   }

//   char angle_msg_buffer[256];
//   for (const auto &pair : servo_data) {
//     const auto r = pair.second;
//     ::snprintf(buf, sizeof(buf) - 1, "%2d %3d p/v/t=(%7.3f,%7.3f,%7.3f)  ", pair.first, static_cast<int>(r.mode), r.position, r.velocity, r.torque);
//     axes[pair.first - 1].curr_pos = (360 * r.position);
//     status_line += buf;
//   }

//   int result = ::snprintf(angle_msg_buffer, sizeof(angle_msg_buffer), "$my_angleP(%f, %f, %f, %f, %f, %f, %f)\n", axes[0].curr_pos,
//                           axes[1].curr_pos, axes[2].curr_pos, axes[3].curr_pos, axes[4].curr_pos, axes[5].curr_pos, axes[6].curr_pos);

//   if (result > 0 && (size_t)result < sizeof(angle_msg_buffer)) {
//     parseArmAngleUart(std::string(angle_msg_buffer));
//   } else {
//     RCLCPP_ERROR(this->get_logger(), "Failed to construct angle message for UART parsing. Result: %d", result);
//   }
// }

// TODO: should send stop cmd after vel cmd so motor doesnt kepp spinning
void ArmCAN::send_velocity_command(float vel[NUM_JOINTS]) {
  std::vector<moteus::CanFdFrame> command_frames;
  float m5, m6;
  differential_drive(vel[AXIS_5_INDEX], vel[AXIS_6_INDEX], m5, m6);
  vel[AXIS_5_INDEX] = m5;
  vel[AXIS_6_INDEX] = m6;

  for (const auto &pair : controllers) {

    moteus::PositionMode::Command position_command;
    const int axis_index = pair.first - 1;

    position_command.position = std::numeric_limits<double>::quiet_NaN();

    // Assuming we want the motor to stop upon reaching the target position.
    RCLCPP_INFO(this->get_logger(), "veloc send %f", static_cast<double>(vel[axis_index]));
    position_command.velocity = vel[axis_index];
    RCLCPP_INFO(this->get_logger(), "vel %f id: %d", position_command.velocity, pair.first);

    motor_telem[axis_index].des_velocity = static_cast<float>(position_command.velocity);
    motor_telem[axis_index].des_position = static_cast<float>(position_command.position);

    if (position_command.velocity == 0.0 && !kHoldTorqueAtZeroVelocity) {
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
  }
}