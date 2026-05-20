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

/**
 * Motor and wheel specs for calculating rescale factors
 */
#define NUM_MOTORS 6
#define MOTOR_GEAR_RATIO 23
#define MOTOR_NUM_POLES 4
#define MOTOR_NUM_PHASES 3
#define WHEEL_RADIUS_METERS 0.1

/**
 * Converts commutations into radians
 * Allows communication with the position controller in radians rather than commutations
 */
#define MOTOR_RESCALE_FACTOR (2.0 * M_PI) / (MOTOR_GEAR_RATIO * MOTOR_NUM_POLES * MOTOR_NUM_PHASES)

/**
 * Time-related constants for loops
 */
#define MOTOR_CONTROL_LOOP_FREQUENCY_MS 50 // Frequency for the main motor control loop
#define DRIVE_FEEDBACK_PUBLISH_FREQUENCY_MS 100 // Publish frequency for drive_feedback_pub_
#define MOTOR_FAILSAFE_INTERVAL_MS 500 // Interval for the Phidget failsafe to shut down the motors

/**
 * Velocity limits
 */
#define MAX_VELOCITY_MS 5
#define MIN_VELOCITY_MS 0.05
#define MAX_VELOCITY_RADS MAX_VELOCITY_MS / WHEEL_RADIUS_METERS

/**
 * Acceleration limits
 */
#define MAX_ACCEL_RADS 10
#define MAX_DV MAX_ACCEL_RADS * (MOTOR_CONTROL_LOOP_FREQUENCY_MS / 1000.0)

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
    MotorControlNode();
    ~MotorControlNode();

private:
    // Publisher for drive feedback (velocity and position of each motor)
    rclcpp::Publisher<rover_msgs::msg::DriveFeedback>::SharedPtr drive_feedback_pub_;
    
    // Subscriber declarations for left and right wheel velocities as Float64MultiArray
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr left_wheel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr right_wheel_sub_;

    // Phidget error handling function
    void handlePhidgetError(PhidgetReturnCode ret, const std::string& action, int motor_id);
    
    // Loop to interact with motors
    void motorControlLoop();

    // Function to record velocity commands for use in the Motor Control Loop
    void setVelocity(const std::vector<int>& selected_motors, float velocity);

    // Callback functions for left and right wheel velocity subscriptions
    void leftWheelCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void rightWheelCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // Function to publish the velocity, target velocity and position of each motor
    void publishDriveFeedback();

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

    // Timer for the main motor control loop
    rclcpp::TimerBase::SharedPtr motor_control_timer_;

    // Motor-related variables
    PhidgetMotorPositionControllerHandle motors[NUM_MOTORS]; // Array of motors
    double target_positions[NUM_MOTORS] = {0.0}; // Array of target motor positions, used as input to the Position Controllers

    /**
     * Arrays for storing velocity values
     * target_velocities: The most recent velocity command received from the controller, clamped to the max velocity
     * applied_velocities: The velocity actually applied to the motor, clamped to the max acceleration in the control loop
     * 
     * The reason for distinction between the 2 arrays is because the velocity limit is applied when the command is received, independent of control loop timing
     * There is no reason to store a velocity command that exceeds the max, so the limit is applied in the setVelocity function so that the clamping is not repeated
     * each time the control loop runs
     * 
     * The acceleration is different each control loop, so it must be applied in the control loop
     * It is saved in applied_velocities for use in the odometry feedback
     */
    double target_velocities[NUM_MOTORS] = {0.0};
    double applied_velocities[NUM_MOTORS] = {0.0};
};

#endif // MOTOR_CONTROL_NODE_H
