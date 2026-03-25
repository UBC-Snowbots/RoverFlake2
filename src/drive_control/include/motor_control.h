#ifndef MOTOR_CONTROL_NODE_H
#define MOTOR_CONTROL_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"  // Correct include for Float64MultiArray message type
#include "rover_msgs/msg/drive_feedback.hpp"
#include "phidget22.h"
#include "rclcpp/qos.hpp"  // Include QoS header
#include <vector>
#include <algorithm>  // For std::clamp

// Motor and wheel specs for calulcating rescale factors
#define MOTOR_GEAR_RATIO 23
#define MOTOR_NUM_POLES 4
#define MOTOR_NUM_PHASES 3

/**
 * Converts commutations into radians
 * Allows communication with the position controller in radians rather than commutations
 */
#define MOTOR_RESCALE_FACTOR (2.0 * M_PI) / (MOTOR_GEAR_RATIO * MOTOR_NUM_POLES * MOTOR_NUM_PHASES)

#define MAX_VELOCITY_MS 5
#define MIN_VELOCITY_MS 0.05
#define WHEEL_RADIUS_METERS 0.1

/**
 * Max velocity of the motors in radians per second
 */
#define MAX_VELOCITY_RADS MAX_VELOCITY_MS / WHEEL_RADIUS_METERS

#define NUM_MOTORS 6
#define MOTOR_CONTROL_LOOP_FREQUENCY_MS 50 // Frequency for the motor control loop
#define DRIVE_FEEDBACK_PUBLISH_FREQUENCY_MS 100 // Publish frequency for drive_feedback_pub_
#define MOTOR_FAILSAFE_INTERVAL_MS 500 // Interval for the Phidget failsafe to shut down the motors (in ms)

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
    
    // Loop to interact with motors
    void motorControlLoop();
    void setVelocity(const std::vector<int>& selected_motors, float velocity);

    // Callback functions for left and right wheel velocity subscriptions
    void leftWheelCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void rightWheelCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // Function to publish the velocity, target velocity and position of each motor
    void publishDriveFeedback();
    void resetFailsafe();

    // Timer to check motor state (velocities and positions) periodically
    rclcpp::TimerBase::SharedPtr feedback_timer_;

    // Timer to reset the motor failsafe
    rclcpp::TimerBase::SharedPtr failsafe_timer_;

    // Timer for the main motor control loop
    rclcpp::TimerBase::SharedPtr motor_control_timer_;

    // Motor-related variables
    PhidgetMotorPositionControllerHandle motors[NUM_MOTORS]; // Array of motors
    double target_positions[NUM_MOTORS] = {0.0}; // Array of target motor position, used as input to the Position Controllers
    double velocities[NUM_MOTORS] = {0.0}; // Array of the current motor velocities, tracked for drive/feedback odometry

    const char* errorString;  // Error string for logging
    char errorDetail[100];  // Detailed error message
    size_t errorDetailLen = 100;  // Length of the error detail buffer
};

#endif // MOTOR_CONTROL_NODE_H
