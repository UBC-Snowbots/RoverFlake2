#include "motor_control.h"

MotorControlNode::MotorControlNode() : Node("motor_control_node") {
    // Initialize connections to the GPIO pins via libgpiod
    chip = gpiod_chip_open_by_name("gpiochip4");
    if (!chip) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open GPIO chip");
        return;
    }
    dir_line = gpio_chip_get_line(chip, DIR_LINE_GPIO_PIN);
    gpiod_line_request_output(dir_line, "stepper_dir", 0);

    step_line = gpio_chip_get_line(chip, STEP_LINE_GPIO_PIN);
    gpiod_line_request_output(step_line, "stepper_line", 0);

    death_ray_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        "death_ray_commands", rclcpp::QoS(10), std::bind(&MotorControlNode::deathRayCommandCallback, this, std::placeholders::_1));
}

MotorControlNode::~MotorControlNode() {
    gpiod_line_release(dir_line);
    gpiod_line_release(step_line);
    gpiod_chip_close(chip);
}

/**
 * Callback for the death_ray_commands subscriber
 * Receives and decides Int16 messages:
 * < 0 -> Counterclockwise
 * > 0 -> Clockwise
 * 
 * In both cases, the magnitude of the command indicates the number of steps to take
 * 
 * This function decodes the command and transmits it to the death ray via the GPIO pins
 */
MotorControlNode::deathRayCommandCallback(const std_msgs::msg::Int16::SharedPtr msg) {
    // Message received from topic will encode the direction in the sign and the num steps in the magnitude
    int stepper_cmd = msg->data;

    // Drive the dir_line using the sign of the command
    if (stepper_cmd > 0) {
        gpio_line_set_value(dir_line, STEPPER_CLOCKWISE_DIRECTION);
    }
    else if (stepper_cmd < 0) {
        gpio_line_set_value(dir_line, !STEPPER_CLOCKWISE_DIRECTION);
    }

    int steps = std::abs(stepper_cmd);

    // Drive the step_line using the magnitude of the command
    for (int i = 0; i < steps; i++) {
        gpiod_line_set_value(step_line, 1);
        std::this_thread::sleep_for(pulse_delay);
        gpiod_line_set_value(step_line, 0);
        std::this_thread::sleep_for(pulse_delay);
    }
}
