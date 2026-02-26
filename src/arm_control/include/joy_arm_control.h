#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joy_feedback.hpp"
#include "rover_msgs/msg/arm_command.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "controller_config.h"

#include <unordered_map>
#include <string>
#include <cmath>

#define NUM_JOINTS 6

#define POSITION_CONTROL 1
#define VELOCITY_CONTROL 2

#define CONTROL_MODE VELOCITY_CONTROL

class ArmJoy : public rclcpp::Node {
public:
    ArmJoy();

private:

// Pubs
rclcpp::Publisher<rover_msgs::msg::ArmCommand>::SharedPtr arm_publisher;
rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr joy_vibrator;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher;

// Subs
rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;
rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr arm_subscriber;
rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_subscriber;

// Callbacks
void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
void arm_callback(const rover_msgs::msg::ArmCommand::SharedPtr msg);
/// Bridge: converts MoveIt Servo JointTrajectory → ArmCommand for the physical arm
void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

// Timers
rclcpp::TimerBase::SharedPtr timer_;

// Helpers
/// Returns true if btn index is valid and pressed
static bool btnPressed(const sensor_msgs::msg::Joy::SharedPtr& msg, int idx);

int current_gear = 0;
int prev_paddleR = 0;
int prev_paddleL = 0;

// Gripper state tracking
bool gripper_open_ = false;
bool prev_gripper_btn_ = false;

// ========== Servo → Physical Arm Bridge ==========
// Maps URDF joint names (from MoveIt Servo) to firmware axis indices (0-5).
std::unordered_map<std::string, int> urdf_to_axis_;
// Axis direction multipliers matching armControlParams.h ArmConstants::axis_dirs.
// Needed to convert between MoveIt (rad/s) and firmware (deg/s) frames.
static constexpr int AXIS_DIR[NUM_JOINTS] = {1, 1, 1, 1, -1, -1};

struct Axis{
    float position = 00.00;
    float velocity = 00.00;
    bool homed = 0;
};
Axis axes[NUM_JOINTS];
};
