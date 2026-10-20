#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <rclcpp/rclcpp.hpp>

#include "rover_msgs/msg/arm_command.hpp"
#include "rover_arm_common/arm_constants.h"
#include "rover_arm_common/arm_commands.h"
#include <chrono>
#include <atomic>
#include <thread>

#include <ArmSerialProtocol.h>
#include <math.h>
#include <serial/serial.h>
#include <array>
#include <deque>
#include <mutex>

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

enum class AxisHomingState : uint8_t { NOT_HOMED = 0, HOMING, HOMED };

struct HomingContext {
  std::array<AxisHomingState, NUM_AXES> axis{}; // value-initialized -> NOT_HOMED
  bool sequence_active = false;                 // firmware accepted a request and hasn't reported completion
};

class ArmSerial : public rclcpp::Node {
public:
  ArmSerial();
  ~ArmSerial();

  rover_msgs::msg::ArmCommand current_arm_status;

  string joint_names[NUM_AXES_NO_EE + 2] = {"joint_1", "joint_2", "joint_3",           "joint_4",
                                        "joint_5", "joint_6", "finger_left_joint", "finger_right_joint"};

  HomingContext getHomingContext() const; // thread-safe snapshot
private:
  unsigned long baud = 115200;
  string port = "/dev/serial/by-id/usb-ZEPHYR_UBC_ROVER_Arm_500100542B2831AA-if00";

  serial::Serial teensy;
  serial::Timeout timeout_uart = serial::Timeout::simpleTimeout(100); // bounds blocking read()/write()

  bool send_angles = true;

  struct Axis {
    float curr_pos;
    float target_pos;
    float speed;
    float zero_rad;
    float max_rad;
    int dir;
  };

  Axis axes[NUM_AXES];

  float target_position[NUM_AXES];
  float target_velocities[NUM_AXES];

  int homed = 0;
  bool homing = false;
  float EE = 0;
  volatile float current_velocity[NUM_AXES] = {00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00};
  volatile float current_position[NUM_AXES] = {00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00};
  volatile int current_limit_switches[NUM_AXES] = {-1, -1, -1, -1, -1, -1, -1};

  rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr command_subscriber;
  void CommandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

  sensor_msgs::msg::JointState prev_joint_states;
  rclcpp::Publisher<rover_msgs::msg::ArmCommand>::SharedPtr arm_position_publisher;

  std::thread serialRxThread;
  std::atomic<bool> rx_running{false};
  std::string rx_accum; // bytes received but not yet terminated by '\n'; only touched by serialRxThread
  HomingContext homing_ctx;               // written only by serialRxThread, from firmware lines
  std::deque<int> pending_home_requests;  // 0 = all axes, 1..NUM_AXES = single axis; awaiting accept/reject
  mutable std::mutex homing_mtx;          // guards homing_ctx and pending_home_requests
  bool handleHomingLine(const std::string &line);


  void serial_rx();
  void processLine(const std::string &line);
  float degToRad(float deg);
  float firmToMoveitOffsetPos(float deg, int axis);
  float firmToMoveitOffsetVel(float deg, int axis);
  void send_position_command(float pos[NUM_AXES]);
  void send_velocity_command(float vel[NUM_AXES]);

  void send_test_limits_command();
  void sendHomeCmd(int target_axis);
  void sendCommCmd(int target_state);
  void sendMsg(std::string outMsg);
  void parseArmAngleUart(std::string msg);
  void parseLimitSwitchTest(std::string msg);
};
