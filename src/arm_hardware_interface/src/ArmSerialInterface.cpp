//TODO: make a good comment
#include "ArmSerialInterface.h"


ArmSerial::ArmSerial() : Node("ArmSerialDriver") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        //command_publisher_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", qos);
        //gear_publisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", qos);

       double period = 1.0/CONTROL_RATE;
        // timer_ = this->create_wall_timer(
        // std::chrono::duration<double>(period),std::bind(&ManualControlNode::test_send, this));
        joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&ArmSerial::joy_callback, this, std::placeholders::_1));
        teensy.setPort(port);
     //   teensy.open();
      //  teensy.setDTR(true);
      //  teensy.setRTS(false);
        sleep(0.1);

        int flag = 1;
        // while(rclcpp::ok()){
        //     recieveMsg();
        // }
    }

void ArmSerial::recieveMsg() {
   std::string next_char = "";
    std::string buffer = "";
    int timeoutCounter = 0;
    //zephyrComm.teensy.flushInput();
   if (teensy.available() > 0){
       // ROS_WARN("Reading");

        //timeoutCounter ++;
       // next_char = teensy.read(); 
        buffer = teensy.read(RX_UART_BUFF);
        RCLCPP_WARN(this->get_logger(), "%s", buffer.c_str());
        // if(next_char == "\n" || next_char == "\r" || next_char == "\0"){
        //     timeoutCounter = RX_UART_BUFF;
        // }
     

// if (buffer.size() > 0){
//         if(buffer.find("Arm Ready") != std::string::npos){
//         homed = true;
//        // fresh_rx_angle = true;
//      }else if(buffer.find("my_angleP") != std::string::npos){
//         parseArmAngleUart(buffer);
//      }


//    }
        //sleep(1);
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
// using ArmSerial();

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmSerial>();

    RCLCPP_INFO(node->get_logger(), "ArmSerial init");

   

    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}