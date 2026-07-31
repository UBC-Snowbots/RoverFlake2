#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"

#include <gpiod.h>

/**
 * Enum to define the control mode of the death ray.
 * For both modes, commands are in the range (-180, 180).
 * 
 * ABS: Absolute commands. The dish is assumed to be at position 0
 *      on node startup (and can be zeroed throughout its runtime).
 *      Commands take the form of absolute positions relative to 0.
 * 
 *      Example: If the dish is at position 20 and you issue the command
 *      50, it will rotate 30 degrees clockwise until it is at 50 degrees.
 * 
 * REL: Relative commands. The node does not need to track where the dish
 *      currently is, commands are simply the number of degrees to move.
 *      
 *      Example: If you issue the command 50, the dish will rotate 50
 *      degrees clockwise.
 */
enum DeathRayControlMode {
    ABS,
    REL
};

/**
 * Define the current control mode of the death ray.
 * Uncomment the appropriate line for the desired mode.
 * 
 * TODO: A way to switch between modes on-the-fly, to better enable
 * automatic death ray control (probably after comp).
 */
#define DEATH_RAY_CONTROL_MODE DeathRayControlMode::ABS
// #define DEATH_RAY_CONTROL_MODE DeathRayControlMode::REL

#define GPIO_CHIP_NAME "/dev/gpiochip0"

// DIR+ pin of the stepper motor. DIR- should be connected to GND.
#define DIR_LINE_GPIO_PIN 23

// PUL+ pin of the stepper motor. PUL- should be connected to GND.
#define STEP_LINE_GPIO_PIN 24

#define STEPPER_CLOCKWISE_DIRECTION 1
#define STEPPER_PULSE_DELAY_MS 2

#define STEPPER_PULSES_PER_REVOLUTION 400.0f
#define STEPPER_GEAR_RATIO 50.0f
#define DISH_PULSES_PER_REVOLUTION (STEPPER_PULSES_PER_REVOLUTION * STEPPER_GEAR_RATIO)
#define DISH_PULSES_PER_DEGREE (DISH_PULSES_PER_REVOLUTION / 360.0f)

#define POSITION_FEEDBACK_PUBLISH_FREQUENCY_MS 200

/**
 * @brief DeathRayMotorControlNode controls the stepper motor to rotate the comms dish
 * 
 * Acts as an interface between the rotation commands and the GPIO output
 */
class DeathRayMotorControlNode : public rclcpp::Node {
public:
    DeathRayMotorControlNode();
    ~DeathRayMotorControlNode();

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr death_ray_motor_sub_;
    void deathRayMotorCallback(const std_msgs::msg::Float32::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr death_ray_zero_sub_;
    void deathRayZeroCallback(const std_msgs::msg::Empty::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr position_feedback_timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr death_ray_position_pub_;
    void publishDeathRayPosition();

    gpiod_chip* chip;
    gpiod_line* dir_line;
    gpiod_line* step_line;

    int position = 0;
    int steps = 0;
};
