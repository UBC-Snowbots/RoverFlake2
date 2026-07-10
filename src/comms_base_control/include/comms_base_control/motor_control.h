#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <gpiod.h>

#define GPIO_CHIP_NAME "/dev/gpiochip0"

// DIR+ pin of the stepper motor. DIR- should be connected to GND.
#define DIR_LINE_GPIO_PIN 23

// PUL+ pin of the stepper motor. PUL- should be connected to GND.
#define STEP_LINE_GPIO_PIN 24

#define STEPPER_CLOCKWISE_DIRECTION 1
#define STEPPER_PULSE_DELAY_MS 100 // TODO: adjust after initial testing

#define STEPPER_PULSES_PER_REVOLUTION 400.0f
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
    void deathRayCommandCallback(const std_msgs::msg::Float32::SharedPtr msg);

    gpiod_chip* chip;
    gpiod_line* dir_line;
    gpiod_line* step_line;

    int steps = 0;
};

#endif
