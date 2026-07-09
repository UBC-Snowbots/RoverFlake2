#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

// This node requires the `libgpiod` package to be installed on the host device
// `sudo apt install libgpiod-dev`
#include <gpiod.h>

#define GPIO_CHIP_NAME "/dev/gpiochip0"

// DIR+ pin of the stepper motor. DIR- should be connected to GND.
#define DIR_LINE_GPIO_PIN 23

// PUL+ pin of the stepper motor. PUL- should be connected to GND.
#define STEP_LINE_GPIO_PIN 24

#define STEPPER_CLOCKWISE_DIRECTION 1
#define STEPPER_PULSE_DELAY_MS 1

#define STEPPER_BASE_PULSES_PER_REVOLUTION 200.0f
/**
 * Determined by SW4,SW5,SW6 on the driver box.
 * 32 corresponds to SW4,SW5 down and SW6 up.
 */
#define DRIVER_MICROSTEPPING_MULTIPLIER 32.0f
#define STEPPER_PULSES_PER_REVOLUTION (STEPPER_BASE_PULSES_PER_REVOLUTION * DRIVER_MICROSTEPPING_MULTIPLIER)
#define STEPPER_GEAR_RATIO 50.0f
#define DISH_PULSES_PER_REVOLUTION (STEPPER_PULSES_PER_REVOLUTION * STEPPER_GEAR_RATIO)
#define DISH_PULSES_PER_DEGREE (DISH_PULSES_PER_REVOLUTION / 360.0f)

/**
 * @brief MotorControlNode controls the stepper motor to rotate the comms dish
 * 
 * Acts as an interface between the rotation commands and the GPIO output
 */
class MotorControlNode : public rclcpp::Node {
public:
    MotorControlNode();
    ~MotorControlNode();

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr death_ray_sub_;
    rclcpp::TimerBase::SharedPtr motor_control_timer_;

    void deathRayCommandCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void motorTimerCallback();

    gpiod_chip* chip;
    gpiod_line* dir_line;
    gpiod_line* step_line;

    int steps = 0;
};

#endif
