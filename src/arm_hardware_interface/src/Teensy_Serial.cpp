#include "ArmSerialInterface.h"

void ArmSerial::setupSerial() {
  teensy.setPort(port);
  teensy.open();
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
  // ROS_INFO("Parsing Angle buffer: %s", msg.c_str());
  sensor_msgs::msg::JointState joint_states_;
  joint_states_.position.resize(NUM_JOINTS);
  joint_states_.velocity.resize(NUM_JOINTS);
  joint_states_.name.resize(NUM_JOINTS);

  if (sscanf(msg.c_str(), "$my_angleP(%f, %f, %f, %f, %f, %f)\n", &axes[0].curr_pos, &axes[1].curr_pos, &axes[2].curr_pos,
             &axes[3].curr_pos, &axes[4].curr_pos, &axes[5].curr_pos) == 6) {
    // All axes angles are in axes[i].des_angle_pos
    RCLCPP_INFO(this->get_logger(), "Absolute Angle Position Echo Accepted:");
    for (int i = 0; i < NUM_JOINTS; i++) {
      current_arm_status.positions[i] = axes[i].curr_pos;
      joint_states_.name[i] = joint_names[i];
      joint_states_.position[i] = firmToMoveitOffsetPos(axes[i].curr_pos, i);
      joint_states_.velocity[i] = firmToMoveitOffsetVel(current_velocity[i], i);
    }
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

void ArmSerial::serial_rx() {
  // rclcpp::Rate loop_rate(50);
  std::string next_char = "";
  std::string buffer = "";
  // int timeoutCounter = 0;
  // zephyrComm.teensy.flushInput();
  if (teensy.available() > 0) {
    // ROS_WARN("Reading");

    // timeoutCounter ++;
    // next_char = teensy.read();
    buffer = teensy.read(RX_UART_BUFF);
    RCLCPP_WARN(this->get_logger(), "got %s", buffer.c_str());
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
// void ArmSerial::sendCommCmd() {
//   //send home command
//   std::string msg = "$h(A)\n";
//   sendMsg(msg);

// }
// using ArmSerial();

// void ArmSerial::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
//     //RCLCPP_INFO(this->get_logger(), "Joy Recieved");
//   if(!homing){
//     int home_button = msg->buttons[2];

//     if(!home_button && homed){
//     target_position[0] = msg->axes[0]*10;
//     target_position[1] = msg->axes[1]*10;
//     target_position[2] = msg->axes[3]*10;
//     target_position[3] = msg->axes[4]*10;
//     target_position[4] = msg->axes[0]*10;
//     target_position[5] = msg->axes[0]*10;
//     send_position_command(target_position);

//     }else{
//         homing = true;
//         sendCommCmd();
//         sendHomeCmd();
//     }
//   }else{

//   }

// }
