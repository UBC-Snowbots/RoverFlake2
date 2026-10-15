//TODO: make a good comment
#include "ArmSerialInterface.h"

#define PI 3.14159

ArmSerial::ArmSerial() : Node("ArmSerialDriver") {
  //? new arm offsets. To change, check arm_control/include/armControlParams.h Should be synced with moveit params

   for(int i = 0; i < NUM_AXES; i++){
    axes[i].zero_rad = ArmConstants::axis_zero_rads[i]; 
    axes[i].dir = ArmConstants::axis_dirs[i];
    #ifdef PRINTOUT_AXIS_PARAMS
      RCLCPP_INFO(this->get_logger(), "Axis %i /// DIR[ %i ] /// OFFSET TO URDF's ZERO_RAD[ %f ] ", i+1, axes[i].dir, axes[i].zero_rad);
    #endif
  }


        // auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(); //Very hack way of only using "live" messages - iffy, and may still operate off of one stale message. in the future we should use a time stamped message, and check the stamp time against current time to make sure msg is not stale.
       auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(); //? AHHHH WHAT THE FUCK IS A QOS

       arm_position_publisher = this->create_publisher<rover_msgs::msg::ArmCommand>("/arm/feedback", qos);
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);

        current_arm_status.positions.resize(NUM_AXES);

        command_subscriber = this->create_subscription<rover_msgs::msg::ArmCommand>(
            "/arm/command", 10, std::bind(&ArmSerial::CommandCallback, this, std::placeholders::_1));

        if(!SIMULATE){
        teensy.setPort(port);
        teensy.setBaudrate(baud);
        teensy.setTimeout(timeout_uart);
        teensy.open();
        rx_running = true;
        serialRxThread = std::thread(&ArmSerial::serial_rx, this);
        }

        sleep(0.1);

        int flag = 1;

    }

ArmSerial::~ArmSerial() {
  rx_running = false;
  if (serialRxThread.joinable()) {
    serialRxThread.join(); // read() times out within timeout_uart, so this returns promptly
  }
  if (teensy.isOpen()) {
    teensy.close();
  }
}

// Entry Point
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmSerial>();

    RCLCPP_INFO(node->get_logger(), "ArmSerial init");
    //std::thread arm_thread(ArmSerial::SerialRxThread, std::ref(node));

     

    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
//Callbacks


void ArmSerial::CommandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg){
  char type = msg->cmd_type;
  switch (type)
  {
  case CMD_HOME:
    if(!SIMULATE){
      sendHomeCmd(msg->cmd_value);
    }
    break;
  case COMM_CMD:
    if(!SIMULATE){
      sendCommCmd(msg->cmd_value);
    }
    break;
  case TEST_LIMITS_CMD:
    if(!SIMULATE){
      send_test_limits_command();
    }
    break;
  case ABS_POS_CMD:
    float target_positions[NUM_AXES];
    for (int i = 0; i < NUM_AXES; i++){
      target_positions[i] = msg->positions[i];
    }
    if(SIMULATE){
       sensor_msgs::msg::JointState joint_states_;
       joint_states_.position.resize(NUM_AXES);
       joint_states_.name.resize(NUM_AXES);
        for(int i = 0; i < NUM_AXES; i++){
        joint_states_.name[i] = joint_names[i];
        joint_states_.position[i] = firmToMoveitOffsetPos(target_positions[i], i);
        joint_states_.velocity[i] = firmToMoveitOffsetVel(current_velocity[i], i);
         }
         joint_states_.header.stamp = rclcpp::Clock().now();

         joint_state_publisher_->publish(joint_states_);

    }else{
          send_position_command(target_positions);

    }
    break;
    case ABS_VEL_CMD:
      // double sim_target_velocities[NUM_AXES];
        for (int i = 0; i < NUM_AXES_NO_EE; i++){
      target_velocities[i] = msg->velocities[i];
      #ifdef DEBUG_MSGS
        RCLCPP_INFO(this->get_logger(), "J%i, %lf", i, msg->velocities[i]);
      #endif// DEBUG_MSGS
      target_velocities[AXIS_EE_INDEX] = msg->end_effector;
      // current_velocity[i] = msg->velocities[i];

    }
     if(SIMULATE){
       sensor_msgs::msg::JointState joint_states_;
      // auto curr_time = this->get_clock()->now();
         joint_states_.header.stamp = rclcpp::Clock().now();

       joint_states_.velocity.resize(NUM_AXES);
       joint_states_.position.resize(NUM_AXES);
       joint_states_.name.resize(NUM_AXES);
        for(int i = 0; i < NUM_AXES; i++){
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

    }else{
   
        send_velocity_command(target_velocities); //!
    }
    break;
  default:
    break;
  
 }
}

// Member functions

float ArmSerial::degToRad(float deg){
  float rad = deg * 3.14159/180; //14159265359

  return(rad);
}

float ArmSerial::firmToMoveitOffsetPos(float deg, int i){

float rad = degToRad(deg);
return ((rad*axes[i].dir) + (axes[i].zero_rad));

}

float ArmSerial::firmToMoveitOffsetVel(float deg, int i){

float rad = degToRad(deg);

return ((rad*axes[i].dir));

}

void ArmSerial::parseLimitSwitchTest(std::string msg){
  int axis = 0;
  int value = 5;
  // Firmware sends: "Limit Switch %d, is %d.  \n"  (comma after the axis number)
  if (sscanf(msg.c_str(), "Limit Switch %d, is %d.", &axis, &value) == 2
      && axis >= 1 && axis <= 6 && (value == 0 || value == 1)) {
    this->current_limit_switches[axis - 1] = value;
    RCLCPP_INFO(this->get_logger(), "Limit switch %d = %d", axis, value);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Arm limit switch parsing failed: %s", msg.c_str());
  }
}

 void ArmSerial::parseArmAngleUart(std::string msg){
     //ROS_INFO("Parsing Angle buffer: %s", msg.c_str());
       sensor_msgs::msg::JointState joint_states_;
       joint_states_.position.resize(NUM_AXES_NO_EE + 2);
       joint_states_.velocity.resize(NUM_AXES_NO_EE + 2);
       joint_states_.name.resize(NUM_AXES_NO_EE + 2);


	float parsed[NUM_AXES];
	if (sscanf(msg.c_str(), "$my_angleP(%f, %f, %f, %f, %f, %f, %f)", &parsed[0], &parsed[1], &parsed[2], &parsed[3], &parsed[4], &parsed[5], &parsed[6]) == NUM_AXES)
	{
		for (int i = 0; i < NUM_AXES; i++) {
			axes[i].curr_pos = parsed[i]; // only commit once all axes parsed
		}
		// All axes angles are in axes[i].des_angle_pos 
		RCLCPP_DEBUG(this->get_logger(), "Absolute Angle Position Echo Accepted");
         for(int i = 0; i < NUM_AXES_NO_EE; i++){
          current_arm_status.positions[i] = axes[i].curr_pos;
          joint_states_.name[i] = joint_names[i];
          joint_states_.position[i] = firmToMoveitOffsetPos(axes[i].curr_pos, i);
          // joint_states_.velocity[i] = firmToMoveitOffsetVel(current_velocity[i], i);
          joint_states_.velocity[i] = firmToMoveitOffsetVel(target_velocities[i], i); //TODO make based off of real velocity from firmware
          // joint_states_.position[i] = (prev_joint_states.position[i]) + (joint_states_.velocity[i] * (current_time - prev_time).seconds());
          // prev_joint_states.position[i] = joint_states_.position[i];
          // joint_states_.position[i] -= axes[i].zero_rad;
         }
             rclcpp::Time current_time(joint_states_.header.stamp);
        rclcpp::Time prev_time(prev_joint_states.header.stamp);
          joint_states_.name[6] = joint_names[6];
          joint_states_.name[7] = joint_names[7];
          joint_states_.position[6] = firmToMoveitOffsetPos(axes[AXIS_EE_INDEX].curr_pos, AXIS_EE_INDEX);
          joint_states_.position[7] = firmToMoveitOffsetPos(axes[AXIS_EE_INDEX].curr_pos * -1, AXIS_EE_INDEX);
          joint_states_.velocity[6] = 0;
          joint_states_.velocity[7] = 0;

         joint_states_.header.stamp = rclcpp::Clock().now();
         arm_position_publisher->publish(current_arm_status);
         joint_state_publisher_->publish(joint_states_);
        //RCLCPP_INFO(this->get_logger(), "Absolute Angle Position Echo Update Successfull");
        //fresh_rx_angle = true;

	}
	else
	{
		// Error handling: could not parse all 6 angles, or message is messed up.
		RCLCPP_ERROR(this->get_logger(), "Absolute Angle Position Echo Rejected, incorrect syntax: %s", msg.c_str());
       // fresh_rx_angle = false;

		return;
	}
 }

void ArmSerial::sendMsg(std::string outMsg) {
    // Send everything in outMsg through serial port
    //ROS_INFO("attempting send");
  //  std::string str_outMsg(reinterpret_cast<const char*>(outMsg), TX_UART_BUFF);
    //std::to_string(str_outMsg);  // +1 to copy the null-terminator
    try {
      size_t written = teensy.write(outMsg);
      if (written != outMsg.size()) {
        RCLCPP_WARN(this->get_logger(), "Serial write short: %zu/%zu bytes", written, outMsg.size());
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Serial write failed: %s", e.what());
    }
}

void ArmSerial::sendHomeCmd(int target_axis) {
  //send home request
    std::string home_msg;
    switch (target_axis)
    {
    case HOME_VALUE_ALL_AXES_EXCEPT_EE:
      home_msg = "$h(A)\n";
      break;
    case HOME_VALUE_ALL_AXES_AND_EE:
      RCLCPP_ERROR(this->get_logger(), "EE Must be homed individually. Ignoring home command. ");
      return;
      break;
    default:
      home_msg = "$h(" + std::to_string(target_axis) + ")\n";
      break;
    }
  sendMsg(home_msg);
}

void ArmSerial::sendCommCmd(int target_state) {
  //send communication request
  std::string msg;
  if(target_state){
    msg = "$SCP(1)\n";
  }else{
    msg = "$SCP(0)\n";
  }
 
  sendMsg(msg);

}


void ArmSerial::serial_rx(){
  // Runs on serialRxThread. Blocks in read() (bounded by timeout_uart) instead of polling.
  while (rx_running && rclcpp::ok()) {
    std::string chunk;
    try {
      size_t n = teensy.available();
      chunk = teensy.read(n > 0 ? n : 1); // blocks for up to timeout_uart waiting for 1 byte
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Serial read failed: %s", e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    if (chunk.empty()) {
      continue;
    }
    rx_accum += chunk;

    // Extract every complete line; keep any trailing partial line for the next read.
    size_t nl;
    while ((nl = rx_accum.find('\n')) != std::string::npos) {
      std::string line = rx_accum.substr(0, nl);
      rx_accum.erase(0, nl + 1);
      processLine(line);
    }

    if (rx_accum.size() > 4 * RX_UART_BUFF) { // no terminator for too long: resync
      RCLCPP_WARN(this->get_logger(), "RX overflow without newline, dropping %zu bytes", rx_accum.size());
      rx_accum.clear();
    }
  }
}

void ArmSerial::processLine(const std::string &raw){
  // Strip '\r', '\0' and spaces left over from "\n\r" terminators or echoed buffers.
  size_t first = raw.find_first_not_of(" \r\t\0", 0, 4);
  if (first == std::string::npos) {
    return; // empty line
  }
  size_t last = raw.find_last_not_of(" \r\t\0", std::string::npos, 4);
  std::string line = raw.substr(first, last - first + 1);

  RCLCPP_DEBUG(this->get_logger(), "RX: %s", line.c_str());

  if (line.find("Arm Ready") != std::string::npos) {
    homed = true;
    homing = false;
  } else if (line.find("$my_angleP") != std::string::npos) {
    parseArmAngleUart(line.substr(line.find('$')));
  } else if (line.find("Limit Switch") != std::string::npos && line.find("Testing") == std::string::npos) {
    parseLimitSwitchTest(line.substr(line.find("Limit Switch")));
  } else {
    RCLCPP_INFO(this->get_logger(), "Teensy: %s", line.c_str());
  }
}

void ArmSerial::send_position_command(float pos[NUM_AXES]) {

    char tx_msg[TX_UART_BUFF];
  
    sprintf(tx_msg, "$P(%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f)\n", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6]);

    sendMsg(tx_msg);
    RCLCPP_INFO(this->get_logger(), "Positions Sent %s", tx_msg);    
}


void ArmSerial::send_velocity_command(float vel[NUM_AXES]) {

    char tx_msg[TX_UART_BUFF];
  
    sprintf(tx_msg, "$V(%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f)\n", vel[0], vel[1], vel[2], vel[3], vel[4], vel[5], vel[6]);
    for(int i = 0; i < NUM_AXES; i++){
      current_velocity[i] = vel[i];
    }
    
    
    sendMsg(tx_msg);
    #ifdef DEBUG_MSGS
    RCLCPP_INFO(this->get_logger(), "Velocities Sent %s", tx_msg);
    #endif //DEBUG_MSGS
}

void ArmSerial::send_test_limits_command(){
  char tx_msg[TX_UART_BUFF];
  sprintf(tx_msg, "$t()\n");
  sendMsg(tx_msg);
  RCLCPP_INFO(this->get_logger(), "Test limits Sent %s", tx_msg);

}