#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "arm_control/include/armControlParams.h"

#include "rover_msgs/msg/arm_command.hpp"
#include <thread>
#include <chrono>

#include <arm_hardware_interface/ArmSerialProtocol.h>


#include <serial/serial.h>

#define SIMULATE false
#define PI 3.14159

#define TX_UART_BUFF 128
#define RX_UART_BUFF 128

#define AXIS_1_DIR 1
#define AXIS_2_DIR 1
#define AXIS_3_DIR 1
#define AXIS_4_DIR 1 
#define AXIS_5_DIR 1
#define AXIS_6_DIR 1





using std::string;



class ArmSerial : public rclcpp::Node {
public:
    ArmSerial();
    void recieveMsg();
    void sendHomeCmd(int target_axis);
    void sendCommCmd(int target_state);
    void sendMsg(std::string outMsg);
    void parseArmAngleUart(std::string msg);
    void parseLimitSwitchTest(std::string msg);
    // float firm

   rover_msgs::msg::ArmCommand current_arm_status;

       //   angle_echo.positions.resize(NUM_JOINTS);
    // void start_rx() {
    //     serialRxThread = std::thread(&ArmSerial::serial_rx(), this);
    // }
    // string joint_names[6] = {"joint_turntable", "joint_axis1", "joint_axis2", "joint_axis3", "joint_axis4", "joint_ender"}; //? old arm urdf
    string joint_names[NUM_JOINTS + 2] = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "finger_left_joint", "finger_right_joint"}; //? newest (Sep 2024) arm urdf


private:

    float degToRad(float deg);
    float firmToMoveitOffsetPos(float deg, int axis);
    float firmToMoveitOffsetVel(float deg, int axis);


   unsigned long baud = 115200;//19200;
    string port = "/dev/serial/by-id/usb-ZEPHYR_UBC_ROVER_Arm_500100C6224069D7-if00";

    serial::Serial teensy;
    serial::Timeout timeout_uart = serial::Timeout::simpleTimeout(1000); // E.g., 1000 ms or 1 second
    

    struct Axis{
      float curr_pos;
      float target_pos;
      float speed;
      float zero_rad;
      float max_rad;
      int dir;
    };

    void CommandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg);

    Axis axes[NUM_JOINTS];
    // Axis axis_EE;


    void send_position_command(float pos[NUM_JOINTS]) {

        char tx_msg[TX_UART_BUFF];
     
        sprintf(tx_msg, "$P(%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f)\n", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6]);

        sendMsg(tx_msg);
        RCLCPP_INFO(this->get_logger(), "Positions Sent %s", tx_msg);
        
    }
    void send_velocity_command(float vel[NUM_JOINTS]) {

        char tx_msg[TX_UART_BUFF];
     
        sprintf(tx_msg, "$V(%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f)\n", vel[0], vel[1], vel[2], vel[3], vel[4], vel[5], vel[6]);
        for(int i = 0; i < NUM_JOINTS; i++){
          current_velocity[i] = vel[i];
        }
        
        
        sendMsg(tx_msg);
        // RCLCPP_INFO(this->get_logger(), "Velocities Sent %s", tx_msg);
        
    }
    void send_test_limits_command(){
      char tx_msg[TX_UART_BUFF];
      sprintf(tx_msg, "$t()\n");
      sendMsg(tx_msg);
      RCLCPP_INFO(this->get_logger(), "Test limits Sent %s", tx_msg);

    }

    void send_gear_command(int gear){
      
    }
  float target_position[NUM_JOINTS];
   float target_velocities[NUM_JOINTS];

    int homed = 0;
    bool homing = false;
    float EE = 0;
    volatile float current_velocity[NUM_JOINTS] = {00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00};
    volatile float current_position[NUM_JOINTS] = {00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00};
    volatile int current_limit_switches[NUM_JOINTS] = {-1, -1, -1, -1, -1, -1, -1};
    rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr command_subscriber;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

    sensor_msgs::msg::JointState prev_joint_states; //for sim
    rclcpp::Publisher<rover_msgs::msg::ArmCommand>::SharedPtr arm_position_publisher;

    rclcpp::TimerBase::SharedPtr timer_;

    

    // void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
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
    std::thread serialRxThread;



    void serial_rx(){
    //rclcpp::Rate loop_rate(50);
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
        if (buffer.size() > 0){
        if(buffer.find("Arm Ready") != std::string::npos){
        homed = true;
        homing = false;
       // fresh_rx_angle = true;
     }else if(buffer.find("my_angleP") != std::string::npos){
        parseArmAngleUart(buffer);
     }else if(buffer.find("Limit Switch")){
        parseLimitSwitchTest(buffer);
     }


   }
    
     

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
    
};




