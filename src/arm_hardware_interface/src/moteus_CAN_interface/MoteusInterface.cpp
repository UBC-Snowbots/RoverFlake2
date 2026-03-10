#include "ArmMoteusInterface.h"

/**
TView commands for moteus:
 * conf set servo.default_timeout_s nan - disable watchdog fault state
**/

float ArmCAN::degToRad(float deg) {
  float rad = deg * M_PI / 180;
  return (rad);
}

float ArmCAN::firmToMoveitOffsetPos(float deg, int i) {
  float rad = degToRad(deg);
  return ((rad * axes[i].dir) + (axes[i].zero_rad));
}

float ArmCAN::firmToMoveitOffsetVel(float deg, int i) {
  float rad = degToRad(deg);
  return ((rad * axes[i].dir));
}

void ArmCAN::parseLimitSwitchTest(std::string msg) { }

void ArmCAN::parseArmAngleUart(std::string msg) {
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
      // RCLCPP_INFO(this->get_logger(), "Curpos %d, %lf", i, current_arm_status.positions[i]);
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

void ArmCAN::serial_rx() {
  if (send_angles) {
    std::vector<moteus::CanFdFrame> command_frames;

    if (homing) {
      send_position_command(home_target_);

      bool all_arrived = true;
      for (int i = 0; i < NUM_JOINTS; i++) {
        double target_rev = radToRev(home_target_[i]);
        double current_rev = motor_telem[i].curr_position;
        if (std::abs(current_rev - target_rev) > kHomePositionThreshold) {
          all_arrived = false;
          break;
        }
      }
      if (all_arrived) {
        homing = false;
        RCLCPP_INFO(this->get_logger(), "Homing complete — all axes reached home position.");
      }
      return;
    }

    // Normal polling: query all motors for telemetry
    for (const auto &pair : controllers) {
      command_frames.push_back(pair.second->MakeQuery());
    }

    if (command_frames.empty()) { return; }

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
      motor_telem[motor_id - 1].moteus_mode = static_cast<int>(result.mode);
      motor_telem[motor_id - 1].moteus_fault = static_cast<int>(result.fault);
      motor_telem[motor_id - 1].curr_position = static_cast<float>(result.position);
      motor_telem[motor_id - 1].curr_velocity = static_cast<float>(result.velocity);
      motor_telem[motor_id - 1].curr_torque_Nm = static_cast<float>(result.torque);
      motor_telem[motor_id - 1].curr_voltage_V = static_cast<float>(result.voltage);
      motor_telem[motor_id - 1].driver_temp_C = static_cast<float>(result.temperature);
      

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

      // RCLCPP_INFO(this->get_logger(), "Gotpos %d, %lf", pair.first, axes[pair.first - 1].curr_pos);
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

  rover_msgs::msg::MoteusArmStatus debug_msg;
  debug_msg.config.resize(NUM_JOINTS);
  debug_msg.status.resize(NUM_JOINTS);
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    debug_msg.config[i].max_acceleration = motor_telem[i].config.max_acceleration;
    debug_msg.config[i].max_velocity = motor_telem[i].config.max_velocity;
    debug_msg.config[i].max_position = motor_telem[i].config.position_max;
    debug_msg.config[i].min_position = motor_telem[i].config.position_min;
    debug_msg.config[i].gear_reduction = motor_telem[i].config.gear_red;

    debug_msg.config[i].kp = motor_telem[i].config.kp;
    debug_msg.config[i].ki = motor_telem[i].config.ki;
    debug_msg.config[i].kd = motor_telem[i].config.kd;

    debug_msg.config[i].max_current_amps = motor_telem[i].config.max_current_A;
    debug_msg.config[i].max_voltage_volts = motor_telem[i].config.max_voltage;
    debug_msg.config[i].max_power_watts = motor_telem[i].config.max_power_W;
    debug_msg.config[i].cmd_timeout_s = motor_telem[i].config.def_timeout;

    debug_msg.status[i].curr_position          = motor_telem[i].curr_position;
    debug_msg.status[i].curr_velocity          = motor_telem[i].curr_velocity;
    debug_msg.status[i].curr_torque            = motor_telem[i].curr_torque_Nm;
    
    debug_msg.status[i].curr_voltage_volts = motor_telem[i].curr_voltage_V;
    debug_msg.status[i].curr_current_amps = motor_telem[i].curr_current_A;
    debug_msg.status[i].curr_power_watts = motor_telem[i].curr_power_W;
    debug_msg.status[i].driver_temp_degreesc = motor_telem[i].driver_temp_C;

    debug_msg.status[i].des_velocity = motor_telem[i].des_velocity;
    debug_msg.status[i].des_position = motor_telem[i].des_position;

    debug_msg.status[i].moteus_mode = motor_telem[i].moteus_mode;
    debug_msg.status[i].moteus_fault = motor_telem[i].moteus_fault;

    
    

  }
  arm_status_publisher->publish(debug_msg);

  checkAlerts();
}

void ArmCAN::checkAlerts()
{
  for (auto &axis : axes)
  {
    
    if(motor_telem[axis.index].curr_position >= motor_telem[axis.index].config.position_max - motor_telem[axis.index].config.position_warn_rev_padding)
    {
      if(axis.alerts.position_alert_raised == false)
      {
        axis.alerts.position_alert_raised = true;
        // Above limit, raise alert and trigger sound
        std_msgs::msg::String msg;
        msg.data = "gary_meow.wav";
        speaker_publisher->publish(msg);
      }
    } 
    else if(motor_telem[axis.index].curr_position <= motor_telem[axis.index].config.position_min +  motor_telem[axis.index].config.position_warn_rev_padding)
    {
      if(axis.alerts.position_alert_raised == false)
      {
        axis.alerts.position_alert_raised = true;
        std_msgs::msg::String msg;
        msg.data = "m-e-o-w.wav";
        speaker_publisher->publish(msg);
      }

    } 
    else 
    {
      axis.alerts.position_alert_raised = false;
    }
  }


}

void ArmCAN::handleWristDifferential(float a5_desired, float a6_desired, float &m5_output, float &m6_output)
{
  m5_output = a6_desired;
  m6_output = a5_desired;

}

void ArmCAN::send_test_limits_command() {
  char tx_msg[TX_UART_BUFF];
  sprintf(tx_msg, "$t()\n");
  sendMsg(tx_msg);
  RCLCPP_INFO(this->get_logger(), "Test limits Sent %s", tx_msg);
}

void ArmCAN::ConfigureMotor(int axis_number, mjbots::moteus::Controller &controller) {
  auto maybe_state = controller.SetQuery();
  if (!maybe_state) {
    RCLCPP_WARN(this->get_logger(), "Motor %d NOT CONNECTED. Skipping config.", axis_number);
    return; // <--- Abort! Don't try to send config commands
  }
  RCLCPP_INFO(this->get_logger(), "Motor %d detected. Starting config...", axis_number);

  std::vector<std::pair<std::string, std::string>> settings = get_arm_configuration()[axis_number - 1].get_configs();


  motor_telem[axis_number - 1].config = get_arm_configuration()[axis_number -1];


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
