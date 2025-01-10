//TODO: make a good comment
#include "ArmSerialInterface.h"

#define PI 3.14159

ArmSerial::ArmSerial() : Node("ArmSerialDriver") {
  //? new arm offsets. To change, check arm_control/include/armControlParams.h Should be synced with moveit params

   for(int i = 0; i < NUM_JOINTS; i++){
    axes[i].zero_rad = ArmConstants::axis_zero_rads[i]; 
    axes[i].dir = ArmConstants::axis_dirs[i];
    #ifdef PRINTOUT_AXIS_PARAMS
      RCLCPP_INFO(this->get_logger(), "Axis %i /// DIR[ %i ] /// OFFSET TO URDF's ZERO_RAD[ %f ] ", i+1, axes[i].dir, axes[i].zero_rad);
    #endif
  }


        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(); //Very hack way of only using "live" messages - iffy, and may still operate off of one stale message. in the future we should use a time stamped message, and check the stamp time against current time to make sure msg is not stale.
        //command_publisher_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", qos);
        //gear_publisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", qos);
        arm_position_publisher = this->create_publisher<rover_msgs::msg::ArmCommand>("/arm/feedback", qos);
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);
       double period = 1.0/COMM_POLL_RATE;

        current_arm_status.positions.resize(NUM_JOINTS);

        command_subscriber = this->create_subscription<rover_msgs::msg::ArmCommand>(
            "/arm/command", 10, std::bind(&ArmSerial::CommandCallback, this, std::placeholders::_1));

        // joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
        //     "/joy", 10, std::bind(&ArmSerial::joy_callback, this, std::placeholders::_1));
        if(!SIMULATE){
        teensy.setPort(port);
        teensy.open();
        timer_ = this->create_wall_timer(
          std::chrono::duration<double>(period),std::bind(&ArmSerial::serial_rx, this));
        }
      //  teensy.setDTR(true);
      //  teensy.setRTS(false);
        sleep(0.1);

        int flag = 1;
        // while(rclcpp::ok()){
        //     recieveMsg();
        // }
        //set serial rx on a quick polling timer
 

    }

void ArmSerial::recieveMsg() {



}

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
  if (sscanf(msg.c_str(), "Limit Switch %d is %d.", &axis, &value) == 2){
    if(axis >= 1 && axis <= 6){
      if(value == 1 || value == 0){
        this->current_limit_switches[axis -1];
        RCLCPP_INFO(this->get_logger(), "limit switches updated");
      }
    }
        RCLCPP_ERROR(this->get_logger(), "Arm limit switch parsing failed");
  }
}

 void ArmSerial::parseArmAngleUart(std::string msg){
     //ROS_INFO("Parsing Angle buffer: %s", msg.c_str());
       sensor_msgs::msg::JointState joint_states_;
       joint_states_.position.resize(NUM_JOINTS);
       joint_states_.velocity.resize(NUM_JOINTS);
       joint_states_.name.resize(NUM_JOINTS);


	if (sscanf(msg.c_str(), "$my_angleP(%f, %f, %f, %f, %f, %f)\n",  &axes[0].curr_pos, &axes[1].curr_pos, &axes[2].curr_pos, &axes[3].curr_pos, &axes[4].curr_pos, &axes[5].curr_pos) == 6)
	{
		// All axes angles are in axes[i].des_angle_pos 
		RCLCPP_INFO(this->get_logger(), "Absolute Angle Position Echo Accepted:");
         for(int i = 0; i < NUM_JOINTS; i++){
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


         joint_states_.header.stamp = rclcpp::Clock().now();
         arm_position_publisher->publish(current_arm_status);
         joint_state_publisher_->publish(joint_states_);
        //RCLCPP_INFO(this->get_logger(), "Absolute Angle Position Echo Update Successfull");
        //fresh_rx_angle = true;

	}
	else
	{
		// Error handling: could not parse all 6 angles, or message is messed up.
		RCLCPP_ERROR(this->get_logger(), "Absolute Angle Position Echo Rejected, incorrect syntax");
       // fresh_rx_angle = false;

		return;
	}


 }

void ArmSerial::sendMsg(std::string outMsg) {
    // Send everything in outMsg through serial port
    //ROS_INFO("attempting send");
  //  std::string str_outMsg(reinterpret_cast<const char*>(outMsg), TX_UART_BUFF);
    //std::to_string(str_outMsg);  // +1 to copy the null-terminator
    teensy.write(outMsg);
  //  RCLCPP_ERROR(this->get_logger(), "Sent via serial: %s", outMsg.c_str());
   teensy.flushOutput();
}

void ArmSerial::sendHomeCmd(int target_axis) {
  //send home request
    std::string home_msg;
  if(target_axis != HOME_ALL_ID){
    home_msg = "$h(" + std::to_string(target_axis) + ")\n";
  }else{
    home_msg = "$h(A)\n";
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

void ArmSerial::CommandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg){
  char type = msg->cmd_type;
  switch (type)
  {
  case HOME_CMD:
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
    float target_positions[NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS; i++){
      target_positions[i] = msg->positions[i];
    }
    if(SIMULATE){
       sensor_msgs::msg::JointState joint_states_;
       joint_states_.position.resize(NUM_JOINTS);
       joint_states_.name.resize(NUM_JOINTS);
        for(int i = 0; i < NUM_JOINTS; i++){
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
      // double sim_target_velocities[NUM_JOINTS];
        for (int i = 0; i < NUM_JOINTS; i++){
      target_velocities[i] = msg->velocities[i];
        RCLCPP_INFO(this->get_logger(), "J%i, %lf", i, msg->velocities[i]);

      // current_velocity[i] = msg->velocities[i];

    }
     if(SIMULATE){
       sensor_msgs::msg::JointState joint_states_;
      // auto curr_time = this->get_clock()->now();
         joint_states_.header.stamp = rclcpp::Clock().now();

       joint_states_.velocity.resize(NUM_JOINTS);
       joint_states_.position.resize(NUM_JOINTS);
       joint_states_.name.resize(NUM_JOINTS);
        for(int i = 0; i < NUM_JOINTS; i++){
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


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmSerial>();

    RCLCPP_INFO(node->get_logger(), "ArmSerial init");
    //std::thread arm_thread(ArmSerial::SerialRxThread, std::ref(node));

     

    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}