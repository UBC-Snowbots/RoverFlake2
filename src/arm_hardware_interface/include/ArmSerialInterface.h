#pragma once

#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <rclcpp/rclcpp.hpp>

#include "rover_msgs/msg/arm_command.hpp"
#include <chrono>
#include <thread>

#include <ArmSerialProtocol.h>

#include "moteus.h"
#include <serial/serial.h>

#define SIMULATE false

#define NUM_JOINTS 6

#define TX_UART_BUFF 128
#define RX_UART_BUFF 128

#define AXIS_1_DIR 1
#define AXIS_2_DIR 1
#define AXIS_3_DIR 1
#define AXIS_4_DIR 1
#define AXIS_5_DIR 1
#define AXIS_6_DIR 1

using std::string;

using namespace mjbots;

class ArmSerial : public rclcpp::Node {
public:
  ArmSerial();
  // void recieveMsg();
  void sendMsg(std::string outMsg);
  void serial_rx();

  void sendHomeCmd(int target_axis);
  void sendCommCmd(int target_state);
  void send_test_limits_command();
  void send_position_command(float pos[NUM_JOINTS]);
  void send_velocity_command(float vel[NUM_JOINTS]);

  void parseArmAngleUart(std::string msg);
  void parseLimitSwitchTest(std::string msg);
  // float firm

  rover_msgs::msg::ArmCommand current_arm_status;

  //   angle_echo.positions.resize(NUM_JOINTS);
  // void start_rx() {
  //     serialRxThread = std::thread(&ArmSerial::serial_rx(), this);
  // }
  // string joint_names[6] = {"joint_turntable", "joint_axis1", "joint_axis2", "joint_axis3", "joint_axis4", "joint_ender"}; //? old arm
  // urdf
  string joint_names[6] = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"}; //? newest (Sep 2024) arm urdf

private:
  float degToRad(float deg);
  float firmToMoveitOffsetPos(float deg, int axis);
  float firmToMoveitOffsetVel(float deg, int axis);

  // unsigned long baud = 19200;
  // string port = "/dev/serial/by-id/usb-ZEPHYR_UBC_ROVER_Arm_500100C6224069D7-if00";
  // string port = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0";
  string port = "/dev/ttyUSB0";
  void setupSerial();
  serial::Serial teensy;
  // serial::Timeout timeout_uart = serial::Timeout::simpleTimeout(1000); // E.g., 1000 ms or 1 second

  std::map<int, std::shared_ptr<moteus::Controller>> controllers;
  std::map<int, moteus::Query::Result> servo_data;
  std::shared_ptr<moteus::Transport> transport;

  struct Axis {
    float curr_pos;
    float target_pos;
    float speed;
    float zero_rad;
    float max_rad;
    int dir;
  };

  void CommandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg);

  Axis axes[NUM_JOINTS];

  // void send_gear_command(int gear) {}
  // float target_position[NUM_JOINTS];

  int homed = 0;
  bool homing = false;
  // float EE = 0;
  volatile float current_velocity[NUM_JOINTS] = {00.00, 00.00, 00.00, 00.00, 00.00, 00.00};
  volatile float current_position[NUM_JOINTS] = {00.00, 00.00, 00.00, 00.00, 00.00, 00.00};
  volatile int current_limit_switches[NUM_JOINTS] = {-1, -1, -1, -1, -1, -1};

  rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr command_subscriber;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<rover_msgs::msg::ArmCommand>::SharedPtr arm_position_publisher;

  rclcpp::TimerBase::SharedPtr timer_;

  // std::thread serialRxThread;
};
