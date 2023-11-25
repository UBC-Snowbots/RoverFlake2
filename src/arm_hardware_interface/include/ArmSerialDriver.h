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

using std::string;

class ArmSerial : public rclcpp::Node {
public:
    ArmSerial() : Node("ArmSerialDriver") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        //command_publisher_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", qos);
        //gear_publisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", qos);


    

        double period = 1.0/CONTROL_RATE;
        // timer_ = this->create_wall_timer(
        // std::chrono::duration<double>(period),std::bind(&ManualControlNode::test_send, this));
        joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&ArmSerial::joy_callback, this, std::placeholders::_1));
    }

    void send_position_command(float pos[NUM_JOINTS]) {
        RCLCPP_INFO(this->get_logger(), "Command Sent");
        
    }

    void send_gear_command(int gear){
      
    }



private:
 //new serial
    unsigned long baud = 19200;
    string port = "/dev/serial/by-id/usb-ZEPHYR_UBC_ROVER_Arm_500100C6224069D7-if00";

    serial::Serial teensy;
    serial::Timeout timeout_uart = serial::Timeout::simpleTimeout(1000); // E.g., 1000 ms or 1 second
    float target_position[NUM_JOINTS];

    float EE = 0;
    float current_position[NUM_JOINTS] = {00.00, 00.00, 00.00, 00.00, 00.00, 00.00};
    // rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr command_publisher_;
    // rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;

    rclcpp::TimerBase::SharedPtr timer_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
        
        target_position[0] = msg->axes[0];
        target_position[1] = msg->axes[1];
        target_position[2] = msg->axes[3];
        target_position[3] = msg->axes[4];
        target_position[4] = msg->axes[0];
        target_position[5] = msg->axes[0];
        RCLCPP_INFO(this->get_logger(), "Joy Recieved");

       
        send_position_command(target_position);
    }
};




