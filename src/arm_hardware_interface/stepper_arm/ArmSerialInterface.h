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
#include <algorithm>
#include <cmath>
#include <limits>

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

// ---------------- Return to base after homing (driver-side, velocity control) ----------------
inline constexpr float NaN_POS = std::numeric_limits<float>::quiet_NaN();
// Firmware-frame angle in degrees (same frame as $my_angleP). NaN = don't move this axis after homing.
inline constexpr float AXIS_BASE_POSITION_DEG[NUM_AXES] = {
    61, // Axis 1 Base
    30, // Axis 2 Shoulder
    20, // Axis 3 Elbow
    125, // Axis 4 Elbow Twist //!warn: Gear red seems off...
    94, // Axis 5 Wrist Pitch
    180, // Axis 6 Wrist Roll
    NaN_POS, // Axis 7 EE
};
inline constexpr float  RETURN_MAX_VEL_DPS[NUM_AXES] = {50, 23, 30, 90, 55, 90, 10}; // per-axis cap, deg/s
inline constexpr float  RETURN_KP              = 6.0f;  // deg/s per degree of error
inline constexpr float  RETURN_MIN_VEL_DPS     = 4.0f;  // floor while outside tolerance, so small errors still move
inline constexpr float  RETURN_TOL_DEG         = 0.1f;  // axis is "at base" within this
inline constexpr double POS_FEEDBACK_TIMEOUT_S = 0.5;   // position older than this counts as not reported
inline constexpr double RETURN_TIMEOUT_S       = 30.0;  // give up and stop after this long
// ---------------- Dance mode (CMD_DANCE 'D') ----------------
// Per axis: target = center + amp * sin(2*pi * (DANCE_BPM/60) * freq_mult * t + phase)
//   amp == 0       -> axis doesn't dance (held with 0 velocity)
//   center == NaN  -> center is wherever the axis is when the dance starts
inline constexpr float  DANCE_BPM = 18.0f;
inline constexpr float  DANCE_CENTER_DEG[NUM_AXES]  = {NaN_POS, NaN_POS, NaN_POS, NaN_POS, NaN_POS, NaN_POS, NaN_POS};
inline constexpr float  DANCE_AMP_DEG[NUM_AXES]     = {40.0f, 10.0f, 10.0f, 60.0f, 10.0f, 0.0f, 0.0f};
inline constexpr float  DANCE_FREQ_MULT[NUM_AXES]   = {0.25f, 0.6666666666f, 0.666666666666f, 0.25f, 1.0f, 1.0f, 1.0f}; // a1: one swing per 4 beats
inline constexpr float  DANCE_PHASE_DEG[NUM_AXES]   = {0.0f, 180.0f, 0.0f, 0.0f, 180.0f, 0.0f, 0.0f}; // elbow opposes shoulder = bob
inline constexpr float  DANCE_MAX_VEL_DPS[NUM_AXES] = {50.0f, 23.0f, 23.0f, 90.0f, 55.0f, 90.0f, 0.0f};
inline constexpr float  DANCE_KP           = 3.0f;  // deg/s per degree of tracking error (on top of feedforward)
inline constexpr float  DANCE_DEADBAND_DEG = 0.3f;  // ignore tracking error below this when the setpoint isn't moving
inline constexpr double DANCE_SETTLE_S     = 3.0;   // time to move to the center pose before swinging
inline constexpr double DANCE_RAMP_S       = 10.0;   // amplitude fade-in and fade-out time
inline constexpr double DANCE_DURATION_S   = 0.0;   // auto wind-down after this long dancing; 0 = until stopped

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

    // Position feedback freshness (written by serialRxThread, read by executor)
  mutable std::mutex feedback_mtx;
  std::array<float, NUM_AXES> fb_pos{};
  std::chrono::steady_clock::time_point fb_stamp{}; // epoch value = never received
  double positionAgeSec() const;                     // +inf if never received
  bool getFreshPosition(std::array<float, NUM_AXES> &out, double max_age_s) const;

  // Motion control (executor thread only, except where noted)
  enum class MotionMode : uint8_t { IDLE, RETURN, DANCE };
  struct AxisSetpoint {
    float pos = NaN_POS;  // NaN = hold this axis (0 velocity)
    float vel_ff = 0.0f;  // feedforward velocity, deg/s
  };
  MotionMode motion_mode = MotionMode::IDLE;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::chrono::steady_clock::time_point mode_start;

  std::array<bool, NUM_AXES> return_pending{}; // axes homed this sequence; guarded by homing_mtx
  std::atomic<bool> return_requested{false};    // set by serialRxThread on "Arm Ready"
  std::array<bool, NUM_AXES> return_axes{};

  std::array<float, NUM_AXES> dance_center{};
  bool dance_stopping = false;
  std::chrono::steady_clock::time_point dance_stop_start;

  void controlStep();
  void positionControl(const std::array<AxisSetpoint, NUM_AXES> &sp, const std::array<float, NUM_AXES> &pos,
                       float kp, const float *max_vel, float min_vel, float deadband, float out_vel[NUM_AXES]) const;
  void startReturn();
  bool startDance();
  void requestDanceStop();
  bool computeDanceSetpoints(double t, std::array<AxisSetpoint, NUM_AXES> &sp);
  void stopMotion(const char *reason);


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
