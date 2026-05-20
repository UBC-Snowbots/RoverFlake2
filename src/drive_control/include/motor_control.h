#ifndef MOTOR_CONTROL_NODE_H
#define MOTOR_CONTROL_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"  // Correct include for Float64MultiArray message type
#include "rover_msgs/msg/drive_feedback.hpp"
#include "rover_msgs/msg/wheel_states.hpp"
#include "std_msgs/msg/bool.hpp"
#include "phidget22.h"
#include "rclcpp/qos.hpp"  // Include QoS header
#include <vector>
#include <algorithm>  // For std::clamp
#include <atomic>
#include <array>
#include <limits>

#define NUM_MOTORS 6
#define DRIVE_FEEDBACK_PUBLISH_FREQUENCY_MS 100 // Publish frequency for drive_feedback_pub_
#define MOTOR_FAILSAFE_INTERVAL_MS 500 // Interval for the Phidget failsafe to shut down the motors (in ms)

#define NUM_WHEELS 6
#define STATUS_HEARTBEAT_INTERVAL_MS 1000

static_assert(NUM_WHEELS == NUM_MOTORS,
    "wheel count and motor count must match — fix WHEEL_TO_PORT before changing");

// Wheel index → Phidget hub port. WheelStates message documents wheel order as
// FL(0), FR(1), ML(2), MR(3), RL(4), RR(5). The existing motor_control treats
// hub ports {3,4,5} as the left side and {0,1,2} as the right side, so:
constexpr int WHEEL_TO_PORT[NUM_WHEELS] = { 3, 0, 4, 1, 5, 2 };

constexpr float TORQUE_PROXY_SCALE = 5.0f;

struct WheelState {
    double prev_pos = 0.0;
    rclcpp::Time prev_time;
    double commanded_vel = 0.0;
    bool engaged = false;
    bool overheat = false;
    bool valid = false;
    bool has_prev = false;
};

class MotorControlNode : public rclcpp::Node {
public:
    MotorControlNode();  // Constructor
    ~MotorControlNode(); // Destructor

private:
    // Publisher for drive feedback (velocity and position of each motor)
    rclcpp::Publisher<rover_msgs::msg::DriveFeedback>::SharedPtr drive_feedback_pub_;
    
    // Subscriber declarations for left and right wheel velocities as Float64MultiArray
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr left_wheel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr right_wheel_sub_;

    // Phidget error handling function
    void handlePhidgetError(PhidgetReturnCode ret, const std::string& action, int motor_id);
    
    // Method to run motors at a specified velocity
    void runMotors(const std::vector<int>& selected_motors, float velocity);

    // Callback functions for left and right wheel velocity subscriptions
    void leftWheelCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void rightWheelCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // Function to publish the velocity, target velocity and position of each motor
    void publishDriveFeedback();
    void resetFailsafe();

    rclcpp::Publisher<rover_msgs::msg::WheelStates>::SharedPtr wheel_states_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_status_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
    rclcpp::TimerBase::SharedPtr stop_heartbeat_timer_;

    std::atomic<bool> remote_stop_{false};
    WheelState wheel_state_[NUM_WHEELS];

    void onRemoteStop(const std_msgs::msg::Bool::SharedPtr msg);
    void publishStopStatus();
    void publishWheelStates();

    // Timer to check motor state (velocities and positions) periodically
    rclcpp::TimerBase::SharedPtr feedback_timer_;

    // Timer to reset the motor failsafe
    rclcpp::TimerBase::SharedPtr failsafe_timer_;

    // Constants and motor-related variables
    PhidgetBLDCMotorHandle motors[NUM_MOTORS];  // Array of motors
    PhidgetReturnCode ret, errorCode;  // Return codes for Phidget functions
    const char* errorString;  // Error string for logging
    char errorDetail[100];  // Detailed error message
    size_t errorDetailLen = 100;  // Length of the error detail buffer
};

#endif // MOTOR_CONTROL_NODE_H
