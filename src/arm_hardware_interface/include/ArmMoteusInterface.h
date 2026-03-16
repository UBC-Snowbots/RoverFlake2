#pragma once

#include "arm_control/include/armControlParams.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"

#include <rclcpp/rclcpp.hpp>
#include "axis_5_6_differential.h"

#include "rover_msgs/msg/arm_command.hpp"
#include "rover_msgs/msg/moteus_arm_status.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "moteus.h"

namespace moteus = mjbots::moteus;

#include <arm_hardware_interface/ArmSerialProtocol.h>
#include <axis_5_6_differential.h>

#define SIMULATE false
#define TX_UART_BUFF 128
#define RX_UART_BUFF 128

#define AXIS_1_DIR 1
#define AXIS_2_DIR 1
#define AXIS_3_DIR 1
#define AXIS_4_DIR 1
#define AXIS_5_DIR 1
#define AXIS_6_DIR 1

using std::string;

class ArmCAN : public rclcpp::Node {
public:
  ArmCAN();

  rover_msgs::msg::ArmCommand current_arm_status;

  string joint_names[NUM_JOINTS + 2] = {
      "joint_1", "joint_2", "joint_3", "joint_4",
      "joint_5", "joint_6", "finger_left_joint", "finger_right_joint"};

private:
  static constexpr bool kHoldTorqueAtZeroVelocity = false;
  static constexpr double kHomeVelocityRevPerSecond = -0.1;
  static constexpr double kHomeMaximumTorqueNm = 0.5;

  struct AxisAlertFlags {
    bool position_alert_raised = false;
    bool current_limit_alert_raised = false;
  };

  struct Axis {
    float curr_pos = 0.0f;
    float target_pos = 0.0f;
    float speed = 0.0f;
    float zero_rad = 0.0f;
    float max_rad = 0.0f;
    int dir = 1;
    int index = -1;
    AxisAlertFlags alerts = {};
  };

  // Motors cannot be linked to axes with new arm (differential wrist)
  struct MotorTelem {
    MotorConfig config;

    float curr_voltage_V = 0.0f;
    float curr_current_A = 0.0f;
    float curr_power_W = 0.0f;
    float driver_temp_C = 0.0f;

    float curr_velocity = 0.0f;
    float curr_position = 0.0f;
    float curr_torque_Nm = 0.0f;

    float des_velocity = 0.0f;
    float des_position = 0.0f;

    int moteus_mode = 0;
    int moteus_fault = 0;
  };

  std::map<int, std::shared_ptr<mjbots::moteus::Controller>> controllers;
  std::map<int, mjbots::moteus::Query::Result> servo_data;
  std::shared_ptr<mjbots::moteus::Transport> transport;

  bool send_angles = true;
  MotorTelem motor_telem[NUM_JOINTS];

  Axis axes[NUM_JOINTS];

  float target_position[NUM_JOINTS] = {0.0f};
  float target_velocities[NUM_JOINTS] = {0.0f};

  int homed = 0;
  bool homing = false;
  float home_target_[NUM_JOINTS] = {0.0f};  // firmware-frame target for homing
  static constexpr float kHomePositionThreshold = 0.01f;  // revolutions — close enough to "arrived"
  float EE = 0.0f;
  volatile float current_velocity[NUM_JOINTS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  volatile float current_position[NUM_JOINTS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  volatile int current_limit_switches[NUM_JOINTS] = {-1, -1, -1, -1, -1, -1, -1};

  rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr command_subscriber;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speaker_publisher;

  sensor_msgs::msg::JointState prev_joint_states;
  rclcpp::Publisher<rover_msgs::msg::ArmCommand>::SharedPtr arm_position_publisher;
  rclcpp::Publisher<rover_msgs::msg::MoteusArmStatus>::SharedPtr arm_status_publisher;

  rclcpp::TimerBase::SharedPtr timer_;

  void CommandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg);
  void serial_rx();

  float degToRad(float deg);
  float firmToMoveitOffsetPos(float deg, int axis);
  float firmToMoveitOffsetVel(float deg, int axis);

  void send_position_command(float pos[NUM_JOINTS]);
  void send_velocity_command(float vel[NUM_JOINTS]);
  void ConfigureMotor(int axis_number, mjbots::moteus::Controller &controller);
  void checkAlerts();
  void handleWristDifferential(float a5_desired, float a6_desired, float &m5_output, float &m6_output);
  void send_test_limits_command();
  void sendHomeCmd();
  void sendCommCmd(int target_state);
  void sendMsg(std::string outMsg);
  void parseArmAngleUart(std::string msg);
  void parseLimitSwitchTest(std::string msg);
};
