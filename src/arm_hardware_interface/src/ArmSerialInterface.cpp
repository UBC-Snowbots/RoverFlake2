//TODO: make a good comment
#include "ArmSerialInterface.h"



ArmSerial::ArmSerial() : Node("ArmSerialDriver") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        //command_publisher_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", qos);
        //gear_publisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", qos);
        arm_position_publisher = this->create_publisher<rover_msgs::msg::ArmCommand>("/arm/status", qos);
       double period = 1.0/COMM_POLL_RATE;

        current_arm_status.positions.resize(NUM_JOINTS);


        joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&ArmSerial::joy_callback, this, std::placeholders::_1));
        teensy.setPort(port);
        teensy.open();
      //  teensy.setDTR(true);
      //  teensy.setRTS(false);
        sleep(0.1);

        int flag = 1;
        // while(rclcpp::ok()){
        //     recieveMsg();
        // }
        timer_ = this->create_wall_timer(
        std::chrono::duration<double>(period),std::bind(&ArmSerial::serial_rx, this));

    }

void ArmSerial::recieveMsg() {



}

 void ArmSerial::parseArmAngleUart(std::string msg){
     //ROS_INFO("Parsing Angle buffer: %s", msg.c_str());
       
	if (sscanf(msg.c_str(), "$my_angleP(%f, %f, %f, %f, %f, %f)\n",  &axes[0].curr_pos, &axes[1].curr_pos, &axes[2].curr_pos, &axes[3].curr_pos, &axes[4].curr_pos, &axes[5].curr_pos) == 6)
	{
		// All axes angles are in axes[i].des_angle_pos
		RCLCPP_INFO(this->get_logger(), "Absolute Angle Position Echo Accepted:");
         for(int i = 0; i < NUM_JOINTS; i++){
        current_arm_status.positions[i] = axes[i].curr_pos;

         }
         arm_position_publisher->publish(current_arm_status);
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
   RCLCPP_ERROR(this->get_logger(), "Sent via serial: %s", outMsg.c_str());
   teensy.flushOutput();
}

void ArmSerial::sendHomeCmd() {
  //send home command
  std::string msg = "$h(A)\n";
  sendMsg(msg);

}

void ArmSerial::sendCommCmd() {
  //send home command
  std::string msg = "$SCP(1)\n";
  sendMsg(msg);

}

// void ArmSerial::sendCommCmd() {
//   //send home command
//   std::string msg = "$h(A)\n";
//   sendMsg(msg);

// }
// using ArmSerial();

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmSerial>();

    RCLCPP_INFO(node->get_logger(), "ArmSerial init");
    //std::thread arm_thread(ArmSerial::SerialRxThread, std::ref(node));

   

    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}