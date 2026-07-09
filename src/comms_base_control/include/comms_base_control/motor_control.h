#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

// This node requires the `libgpiod` package to be installed on the host device
// `sudo apt install libgpiod-dev`
#include <gpiod.h>

#define GPIO_CHIP_NAME "/dev/gpiochip0"
#define DIR_LINE_GPIO_PIN 23
#define STEP_LINE_GPIO_PIN 24
#define EN_LINE_GPIO_PIN 25
#define STEPPER_CLOCKWISE_DIRECTION 1
#define STEPPER_PULSE_DELAY_MS 2 

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
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr death_ray_sub_;
    void deathRayCommandCallback(const std_msgs::msg::Int16::SharedPtr msg);

    gpiod_chip* chip;
    gpiod_line* dir_line;
    gpiod_line* step_line;
    gpiod_line* en_line;
};

#endif
