#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joy_feedback.hpp"
#include "rover_msgs/msg/arm_command.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "controller_config.h"

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

// Callbacks
void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
void arm_callback(const rover_msgs::msg::ArmCommand::SharedPtr msg);

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

struct Axis{
    float position = 00.00;
    float velocity = 00.00;
    bool homed = 0;
};
Axis axes[NUM_JOINTS];
};
