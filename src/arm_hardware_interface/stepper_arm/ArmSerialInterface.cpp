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
        control_timer_ = this->create_wall_timer(
          std::chrono::duration<double>(1.0 / CONTROL_RATE),
          std::bind(&ArmSerial::controlStep, this));
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
      if (motion_mode != MotionMode::IDLE) stopMotion("home requested");
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
        if (motion_mode != MotionMode::IDLE) {
          bool nonzero = false;
          for (int i = 0; i < NUM_AXES; i++) {
            if (std::fabs(target_velocities[i]) > 1e-3f) nonzero = true;
          }
          if (!nonzero) break;                     // idle teleop zeros: keep going
          stopMotion("operator velocity command"); // operator takes over
        }
        send_velocity_command(target_velocities); //!
    }
    break;
    case CMD_DANCE:
    if (!SIMULATE) {
      if (msg->cmd_value != 0) {
        startDance();
      } else {
        requestDanceStop();
      }
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
    {
			std::lock_guard<std::mutex> lk(feedback_mtx);
			for (int i = 0; i < NUM_AXES; i++) fb_pos[i] = parsed[i];
			fb_stamp = std::chrono::steady_clock::now();
		}
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
  std::string home_msg;
  int pending = 0;
  switch (target_axis)
  {
  case HOME_VALUE_ALL_AXES_EXCEPT_EE:
    home_msg = "$h(A)\n";
    pending = 0;
    break;
  case HOME_VALUE_ALL_AXES_AND_EE:
    RCLCPP_ERROR(this->get_logger(), "EE Must be homed individually. Ignoring home command. ");
    return;
  default:
    if (target_axis < 0 || target_axis > (NUM_AXES - 1)) { // firmware parses a single digit, 1-based, but we use 0-based index accross ros topics. its annoying...
      RCLCPP_ERROR(this->get_logger(), "Home request for invalid axis %d ignored", target_axis);
      return;
    }
    home_msg = "$h(" + std::to_string(target_axis + 1) + ")\n";
    pending = target_axis;
    break;
  }
  {
    // Record before sending so the firmware's reply can't arrive first.
    // This does NOT change any axis state; only a firmware accept does.
    std::lock_guard<std::mutex> lk(homing_mtx);
    pending_home_requests.push_back(pending);
    while (pending_home_requests.size() > 8) pending_home_requests.pop_front();
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

  if (line.find("$my_angleP") != std::string::npos) {       // hot path first (100 Hz)
    parseArmAngleUart(line.substr(line.find('$')));
  } else if (handleHomingLine(line)) {
    RCLCPP_INFO(this->get_logger(), "Teensy: %s", line.c_str());
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

static const char *homingStateStr(AxisHomingState s) {
  switch (s) {
    case AxisHomingState::NOT_HOMED: return "NOT_HOMED";
    case AxisHomingState::HOMING:    return "HOMING";
    case AxisHomingState::HOMED:     return "HOMED";
  }
  return "?";
}

HomingContext ArmSerial::getHomingContext() const {
  std::lock_guard<std::mutex> lk(homing_mtx);
  return homing_ctx;
}

// Called from serialRxThread for every line. Returns true if the line was a homing message.
bool ArmSerial::handleHomingLine(const std::string &line) {
  std::lock_guard<std::mutex> lk(homing_mtx);

  auto popPending = [this]() -> int {
    if (pending_home_requests.empty()) return -1;
    int r = pending_home_requests.front();
    pending_home_requests.pop_front();
    return r;
  };
  auto logCtx = [this]() {
    std::string s;
    for (int i = 0; i < NUM_AXES; i++) {
      s += "A" + std::to_string(i + 1) + "=" + homingStateStr(homing_ctx.axis[i]) + " ";
    }
    RCLCPP_INFO(this->get_logger(), "Homing ctx: %s%s", s.c_str(),
                homing_ctx.sequence_active ? "(sequence active)" : "");
  };

  int axis_num = 0;

  // Boot banner "Axis N attached at STEP: ..." -> firmware restarted, nothing is homed.
  if (line.rfind("Axis ", 0) == 0 && line.find(" attached at ") != std::string::npos) {
    homing_ctx = HomingContext{};
    pending_home_requests.clear();
    return_pending.fill(false);
    return true;
  }

  // Accepted: $h(A)
  if (line.find("Homing All axes") != std::string::npos) {
    popPending();
    for (int i = 0; i < NUM_AXES_NO_EE; i++) homing_ctx.axis[i] = AxisHomingState::HOMING;
    homing_ctx.sequence_active = true;
    logCtx();
    return true;
  }

  // Accepted: $h(N). Firmware doesn't echo N, so use the oldest outstanding request.
  if (line.find("Specific Axis Requested") != std::string::npos) {
    int req = popPending();
    if (req >= 1 && req <= NUM_AXES) {
      homing_ctx.axis[req - 1] = AxisHomingState::HOMING;
    } else {
      RCLCPP_WARN(this->get_logger(), "Firmware accepted a single-axis home, but no matching request is pending");
    }
    homing_ctx.sequence_active = true;
    logCtx();
    return true;
  }

  // Rejected: drop the request, leave all states unchanged.
  if (line.find("Homing Sequence In progress") != std::string::npos ||
      line.find("Homing Request Denied") != std::string::npos ||
      line.find("Error parsing requested_axis") != std::string::npos) {
    int req = popPending();
    RCLCPP_WARN(this->get_logger(), "Firmware rejected home request (%d, 0=all): %s", req, line.c_str());
    return true;
  }

  // Per-axis completion: "Axis N is home."
  if (line.find(" is home") != std::string::npos &&
      sscanf(line.c_str(), "Axis %d is home", &axis_num) == 1 &&
      axis_num >= 1 && axis_num <= NUM_AXES) {
    homing_ctx.axis[axis_num - 1] = AxisHomingState::HOMED;
    return_pending[axis_num - 1] = true;
    logCtx();
    return true;
  }

  // Sequence finished. Firmware marks the EE homed without ever sending "Axis 7 is home",
  // so anything still HOMING here is homed as far as the firmware is concerned.
  if (line.find("Homing complete") != std::string::npos || line.find("Arm Ready") != std::string::npos) {
      for (int i = 0; i < NUM_AXES; i++) {
      if (homing_ctx.axis[i] == AxisHomingState::HOMING) {
        homing_ctx.axis[i] = AxisHomingState::HOMED;
        return_pending[i] = true;
      }
    }
    homing_ctx.sequence_active = false;
    homed = true;     // keep legacy flags in sync
    homing = false;
    logCtx();
    // "Arm Ready" is sent right before the firmware clears arm_homing, so $V is accepted after this.
    if (line.find("Arm Ready") != std::string::npos) {
      return_requested = true;
    }
    return true;
  }

  return false;
}


double ArmSerial::positionAgeSec() const {
  std::lock_guard<std::mutex> lk(feedback_mtx);
  if (fb_stamp.time_since_epoch().count() == 0) return std::numeric_limits<double>::infinity();
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - fb_stamp).count();
}

bool ArmSerial::getFreshPosition(std::array<float, NUM_AXES> &out, double max_age_s) const {
  std::lock_guard<std::mutex> lk(feedback_mtx);
  if (fb_stamp.time_since_epoch().count() == 0) return false;
  double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - fb_stamp).count();
  if (age > max_age_s) return false;
  out = fb_pos;
  return true;
}
// Position mode: velocity = feedforward + kp * error, clamped to +-max_vel[i].
// NaN setpoint -> 0 velocity. min_vel (if > 0) is a floor applied only when there's no feedforward.
void ArmSerial::positionControl(const std::array<AxisSetpoint, NUM_AXES> &sp, const std::array<float, NUM_AXES> &pos,
                                float kp, const float *max_vel, float min_vel, float deadband,
                                float out_vel[NUM_AXES]) const {
  for (int i = 0; i < NUM_AXES; i++) {
    out_vel[i] = 0.0f;
    if (std::isnan(sp[i].pos)) continue;

    float err = sp[i].pos - pos[i];
    bool moving_sp = std::fabs(sp[i].vel_ff) > 1e-3f;
    if (!moving_sp && std::fabs(err) <= deadband) continue;

    float v = sp[i].vel_ff + kp * err;
    if (!moving_sp && min_vel > 0.0f && std::fabs(v) < min_vel) v = std::copysign(min_vel, err);
    out_vel[i] = std::clamp(v, -max_vel[i], max_vel[i]);
  }
}

void ArmSerial::stopMotion(const char *reason) {
  const char *name = (motion_mode == MotionMode::RETURN) ? "Return to base"
                   : (motion_mode == MotionMode::DANCE)  ? "Dance" : "Motion";
  float zero[NUM_AXES] = {0};
  send_velocity_command(zero);
  motion_mode = MotionMode::IDLE;
  return_axes.fill(false);
  dance_stopping = false;
  if (reason) {
    RCLCPP_WARN(this->get_logger(), "%s aborted: %s", name, reason);
  } else {
    RCLCPP_INFO(this->get_logger(), "%s complete", name);
  }
}

void ArmSerial::startReturn() {
  std::array<bool, NUM_AXES> req{};
  {
    std::lock_guard<std::mutex> lk(homing_mtx);
    req = return_pending;
    return_pending.fill(false);
  }
  bool any = false;
  for (int i = 0; i < NUM_AXES; i++) {
    return_axes[i] = req[i] && !std::isnan(AXIS_BASE_POSITION_DEG[i]);
    any = any || return_axes[i];
  }
  if (!any) return;

  std::array<float, NUM_AXES> tmp;
  if (!getFreshPosition(tmp, POS_FEEDBACK_TIMEOUT_S)) {
    RCLCPP_ERROR(this->get_logger(),
                 "Return to base skipped: no fresh position feedback (age %.2f s). Is $SCP(1) on?",
                 positionAgeSec());
    return_axes.fill(false);
    return;
  }
  motion_mode = MotionMode::RETURN;
  mode_start = std::chrono::steady_clock::now();
  RCLCPP_INFO(this->get_logger(), "Returning homed axes to base position");
}

bool ArmSerial::startDance() {
  if (motion_mode != MotionMode::IDLE) {
    RCLCPP_WARN(this->get_logger(), "Dance refused: arm is busy");
    return false;
  }
  std::array<float, NUM_AXES> pos;
  if (!getFreshPosition(pos, POS_FEEDBACK_TIMEOUT_S)) {
    RCLCPP_ERROR(this->get_logger(), "Dance refused: no fresh position feedback (age %.2f s). Is $SCP(1) on?",
                 positionAgeSec());
    return false;
  }
  HomingContext hc = getHomingContext();
  bool any = false;
  for (int i = 0; i < NUM_AXES; i++) {
    if (DANCE_AMP_DEG[i] == 0.0f) continue;
    if (hc.axis[i] != AxisHomingState::HOMED) {
      RCLCPP_ERROR(this->get_logger(), "Dance refused: axis %d is not homed", i + 1);
      return false;
    }
    dance_center[i] = std::isnan(DANCE_CENTER_DEG[i]) ? pos[i] : DANCE_CENTER_DEG[i];
    float w = 2.0f * float(M_PI) * (DANCE_BPM / 60.0f) * DANCE_FREQ_MULT[i];
    float peak = std::fabs(DANCE_AMP_DEG[i]) * w;
    if (peak > DANCE_MAX_VEL_DPS[i]) {
      RCLCPP_WARN(this->get_logger(), "Dance: axis %d needs %.0f deg/s peak, capped at %.0f; motion will be clipped",
                  i + 1, peak, DANCE_MAX_VEL_DPS[i]);
    }
    any = true;
  }
  if (!any) {
    RCLCPP_WARN(this->get_logger(), "Dance refused: no axes have a nonzero DANCE_AMP_DEG");
    return false;
  }
  dance_stopping = false;
  motion_mode = MotionMode::DANCE;
  mode_start = std::chrono::steady_clock::now();
  RCLCPP_INFO(this->get_logger(), "Dance starting (settling to center for %.1f s)", DANCE_SETTLE_S);
  return true;
}

void ArmSerial::requestDanceStop() {
  if (motion_mode != MotionMode::DANCE || dance_stopping) return;
  dance_stopping = true;
  dance_stop_start = std::chrono::steady_clock::now();
  RCLCPP_INFO(this->get_logger(), "Dance winding down");
}

// Fills dance setpoints for time t (seconds since the dance started). Returns false when the dance is finished.
bool ArmSerial::computeDanceSetpoints(double t, std::array<AxisSetpoint, NUM_AXES> &sp) {
  const double ramp = std::max(DANCE_RAMP_S, 1e-3);
  double td = t - DANCE_SETTLE_S;                        // time spent actually dancing (<0 while settling)
  double env = (td <= 0.0) ? 0.0 : std::min(1.0, td / ramp);

  if (DANCE_DURATION_S > 0.0 && td >= DANCE_DURATION_S) requestDanceStop();
  if (dance_stopping) {
    double ts = std::chrono::duration<double>(std::chrono::steady_clock::now() - dance_stop_start).count();
    double fade = 1.0 - ts / ramp;
    if (fade <= 0.0) return false;
    env = std::min(env, fade);
  }

  for (int i = 0; i < NUM_AXES; i++) {
    if (DANCE_AMP_DEG[i] == 0.0f) continue;              // NaN setpoint -> held at 0 velocity
    double w = 2.0 * M_PI * (DANCE_BPM / 60.0) * DANCE_FREQ_MULT[i];
    double ph = w * std::max(td, 0.0) + DANCE_PHASE_DEG[i] * M_PI / 180.0;
    sp[i].pos = float(dance_center[i] + env * DANCE_AMP_DEG[i] * std::sin(ph));
    sp[i].vel_ff = (td > 0.0) ? float(env * DANCE_AMP_DEG[i] * w * std::cos(ph)) : 0.0f;
  }
  return true;
}

void ArmSerial::controlStep() {
  if (return_requested.exchange(false)) {
    if (motion_mode == MotionMode::IDLE) {
      startReturn();
    } else {
      std::lock_guard<std::mutex> lk(homing_mtx);
      return_pending.fill(false);
    }
  }
  if (motion_mode == MotionMode::IDLE) return;

  std::array<float, NUM_AXES> pos;
  if (!getFreshPosition(pos, POS_FEEDBACK_TIMEOUT_S)) {
    stopMotion("position feedback went stale");
    return;
  }
  double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - mode_start).count();

  std::array<AxisSetpoint, NUM_AXES> sp{}; // default: every axis held
  float vel[NUM_AXES];

  if (motion_mode == MotionMode::RETURN) {
    if (t > RETURN_TIMEOUT_S) {
      stopMotion("timed out");
      return;
    }
    bool all_done = true;
    for (int i = 0; i < NUM_AXES; i++) {
      if (!return_axes[i]) continue;
      if (std::fabs(AXIS_BASE_POSITION_DEG[i] - pos[i]) <= RETURN_TOL_DEG) {
        return_axes[i] = false; // latch done so it doesn't hunt around the target
        RCLCPP_INFO(this->get_logger(), "Axis %d at base (%.2f deg)", i + 1, pos[i]);
        continue;
      }
      all_done = false;
      sp[i].pos = AXIS_BASE_POSITION_DEG[i];
    }
    if (all_done) {
      stopMotion(nullptr);
      return;
    }
    positionControl(sp, pos, RETURN_KP, RETURN_MAX_VEL_DPS, RETURN_MIN_VEL_DPS, 0.0f, vel);
  } else { // DANCE
    if (!computeDanceSetpoints(t, sp)) {
      stopMotion(nullptr);
      return;
    }
    positionControl(sp, pos, DANCE_KP, DANCE_MAX_VEL_DPS, 0.0f, DANCE_DEADBAND_DEG, vel);
  }
  send_velocity_command(vel);
}