#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joy.hpp"


#include <serial/serial.h>

#define NUM_JOINTS 6

#define TX_UART_BUFF 128
#define RX_UART_BUFF 128

#define AXIS_1_DIR 1
#define AXIS_2_DIR 1
#define AXIS_3_DIR 1
#define AXIS_4_DIR 1
#define AXIS_5_DIR 1
#define AXIS_6_DIR 1


#define CONTROL_RATE 60.0
#define GEAR_REVERSE 20
#define GEAR_PARKING 22
#define GEAR_NEUTRAL 1
#define GEAR_1 2 //or drive
#define GEAR_2 3 //guessing this is how autoware deals with manual cars

#define RX_UART_BUFF 90

using std::string;

class ArmSerial : public rclcpp::Node {
public:
    ArmSerial();
    void recieveMsg();
    void sendHomeCmd();
    void sendMsg(std::string outMsg);



private:
   unsigned long baud = 19200;
    string port = "/dev/serial/by-id/usb-ZEPHYR_UBC_ROVER_Arm_500100C6224069D7-if00";

    serial::Serial teensy;
    serial::Timeout timeout_uart = serial::Timeout::simpleTimeout(1000); // E.g., 1000 ms or 1 second
   
   

    void send_position_command(float pos[NUM_JOINTS]) {

        char tx_msg[TX_UART_BUFF];
     
        sprintf(tx_msg, "$P(%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f)\n", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);

        sendMsg(tx_msg);
        RCLCPP_INFO(this->get_logger(), "Command Sent %s", tx_msg);
        
    }

    void send_gear_command(int gear){
      
    }
  float target_position[NUM_JOINTS];

    int homed = 0;
    float EE = 0;
    float current_position[NUM_JOINTS] = {00.00, 00.00, 00.00, 00.00, 00.00, 00.00};
   
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;

    rclcpp::TimerBase::SharedPtr timer_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
        //RCLCPP_INFO(this->get_logger(), "Joy Recieved");

        int home_button = msg->buttons[2];

        if(!home_button && homed){
        target_position[0] = msg->axes[0];
        target_position[1] = msg->axes[1];
        target_position[2] = msg->axes[3];
        target_position[3] = msg->axes[4];
        target_position[4] = msg->axes[0];
        target_position[5] = msg->axes[0];
        send_position_command(target_position);

        }else{
            homed = 1;
            sendHomeCmd();
        }
       
    }
};




