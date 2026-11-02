#ifndef MOTOR_CONTROL_NODE_H
#define MOTOR_CONTROL_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rover_msgs/msg/drive_feedback.hpp"
#include "moteus.h" // mjbots moteus C++ client (src/external_pkgs/moteus)

#include <algorithm> // For std::clamp
#include <cmath>     // For M_PI, NAN
#include <memory>
#include <vector>

namespace mot = mjbots::moteus;

/**
 * Wheel specs for unit conversions
 *
 * The moteus controllers speak OUTPUT-SHAFT revolutions and rev/s on the
 * wire. The gearbox reduction is handled by the moteus firmware itself via
 * `motor_position.rotor_to_output_ratio`, configured on each controller with
 * moteus_tool. No gear-ratio math is needed on the host side.
 */
#define NUM_MOTORS 6
#define WHEEL_RADIUS_METERS 0.1
#define WHEEL_CIRCUMFERENCE_METERS (2.0 * M_PI * WHEEL_RADIUS_METERS)

/**
 * Unit conversions
 * Wheel surface speed (m/s) <-> output-shaft rev/s, and rev <-> rad for the
 * odometry feedback message (which reports positions in radians).
 */
#define MS_TO_REVS (1.0 / WHEEL_CIRCUMFERENCE_METERS)
#define REVS_TO_RADS (2.0 * M_PI)

/**
 * CAN addressing
 *
 * Motor index i (0-5) maps to moteus CAN ID i + DRIVE_MOTOR_CAN_ID_BASE.
 * Right wheels: indices 0-2, left wheels: indices 3-5.
 *
 * NOTE: the arm (arm_hardware_interface/moteus_driver) also uses CAN IDs 1-6.
 * This node and the arm driver must NOT run on the same CAN bus at the same
 * time until one of them is moved to a different ID range.
 */
#define DRIVE_MOTOR_CAN_ID_BASE 1

/**
 * Time-related constants for loops
 *
 * The moteus watchdog (`servo.default_timeout_s`, 0.1 s by default) stops the
 * motors automatically if command frames stop arriving, replacing the Phidget
 * failsafe. The control loop below re-sends commands well within that window.
 */
#define MOTOR_CONTROL_LOOP_FREQUENCY_MS 50       // Frequency for the main motor control loop
#define DRIVE_FEEDBACK_PUBLISH_FREQUENCY_MS 1000 // Publish frequency for drive_feedback_pub_

/**
 * Velocity limits
 */
#define MAX_VELOCITY_MS 5.5
#define MIN_VELOCITY_MS 0.05
#define MAX_VELOCITY_REVS (MAX_VELOCITY_MS * MS_TO_REVS)

/**
 * Acceleration limit (output-shaft rev/s^2) — same 50 rad/s^2 cap as the old
 * Phidget setup. Enforced by the moteus motion profile via accel_limit.
 */
#define MAX_ACCEL_REVS (50.0 / (2.0 * M_PI))

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

    // Main loop: sends velocity commands to all motors in one CAN cycle and
    // decodes the telemetry replies
    void motorControlLoop();

    // Function to record target velocity for use in the motor control loop
    void setVelocity(const std::vector<int>& selected_motors, float velocity);

    // Callback functions for left and right wheel velocity subscriptions
    void leftWheelCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void rightWheelCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // Function to publish the target/actual position and velocity of each motor for odometry
    void publishDriveFeedback();

    // Timer to publish odometry (velocities and positions) periodically
    rclcpp::TimerBase::SharedPtr feedback_timer_;

    // Timer for the main motor control loop
    rclcpp::TimerBase::SharedPtr motor_control_timer_;

    // CAN transport shared by all controllers
    std::shared_ptr<mot::Transport> transport_;

    // One moteus controller handle per motor (CAN IDs DRIVE_MOTOR_CAN_ID_BASE + i)
    std::vector<std::shared_ptr<mot::Controller>> controllers_;

    // The most recent velocity command per motor, in output-shaft rev/s,
    // clamped to the max velocity
    double target_velocities_revs_[NUM_MOTORS] = {0.0};

    // Commanded position integrated from target velocity (radians), reported
    // in DriveFeedback.target_positions for odometry consumers
    double target_positions_rad_[NUM_MOTORS] = {0.0};

    // Latest telemetry reply per motor and whether one arrived last cycle
    mot::Query::Result telem_[NUM_MOTORS];
    bool connected_[NUM_MOTORS] = {false};
};

#endif // MOTOR_CONTROL_NODE_H
