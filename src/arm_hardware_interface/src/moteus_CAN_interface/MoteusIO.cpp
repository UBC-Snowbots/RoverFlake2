#include "ArmMoteusInterface.h"

// This file does shit

void ArmCAN::send_position_command(float pos[NUM_JOINTS]) {
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
void ArmCAN::send_velocity_command(float vel[NUM_JOINTS]) {
  std::vector<moteus::CanFdFrame> command_frames;

  std::map<int, double> target_positions;

  // convert deg/s to revolutions/s
  const double DEG_TO_REVOLUTIONS = 1.0 / 360.0;

  differential_drive(vel[AXIS_5_INDEX], vel[AXIS_6_INDEX], vel[AXIS_5_INDEX], vel[AXIS_6_INDEX]);


  // Accumulate all of our command CAN frames.
  for (const auto &pair : controllers) {
    // pair.first is the CAN ID
    // pair.second is the moteus::Controller object

    moteus::PositionMode::Command position_command;

    position_command.position = std::numeric_limits<double>::quiet_NaN();

    // Assuming we want the motor to stop upon reaching the target position.
    RCLCPP_INFO(this->get_logger(), "veloc send %f", static_cast<double>(vel[(pair.first)]));
    position_command.velocity = vel[pair.first - 1];
    // position_command.velocity = vel[pair.first - 1];
    RCLCPP_INFO(this->get_logger(), "vel %f id: %d", position_command.velocity, pair.first);

    if (position_command.velocity == 0 && HOLD_TORQUE_AT_ZERO_VELOCITY == false) {
      RCLCPP_INFO(this->get_logger(), "velocity stop %f", position_command.velocity);
      pair.second->SetStop();
    } else {
      if(pair.first != 4)
      {
          command_frames.push_back(pair.second->MakePosition(position_command));
      }
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