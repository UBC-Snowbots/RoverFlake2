#include "death_ray_motor_control.h"

DeathRayMotorControlNode::DeathRayMotorControlNode() : Node("death_ray_motor_control_node") {
    RCLCPP_INFO(this->get_logger(), "Attempting to open GPIO chip: %s", GPIO_CHIP_NAME);

    // Initialize connections to the GPIO pins via libgpiod
    chip = gpiod_chip_open(GPIO_CHIP_NAME);
    if (!chip) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open GPIO chip: %s", GPIO_CHIP_NAME);
        return;
    }

    dir_line = gpiod_chip_get_line(chip, DIR_LINE_GPIO_PIN);
    if (!dir_line) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get DIR line on pin %d", DIR_LINE_GPIO_PIN);
    } else {
        gpiod_line_request_output(dir_line, "stepper_dir", 0);
        RCLCPP_INFO(this->get_logger(), "Successfully requested DIR line on pin %d", DIR_LINE_GPIO_PIN);
    }

    step_line = gpiod_chip_get_line(chip, STEP_LINE_GPIO_PIN);
    if (!step_line) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get STEP line on pin %d", STEP_LINE_GPIO_PIN);
    } else {
        gpiod_line_request_output(step_line, "stepper_line", 0);
        RCLCPP_INFO(this->get_logger(), "Successfully requested STEP line on pin %d", STEP_LINE_GPIO_PIN);
    }

    death_ray_motor_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "death_ray/motor_commands", rclcpp::QoS(10), std::bind(&DeathRayMotorControlNode::deathRayMotorCallback, this, std::placeholders::_1));

    death_ray_zero_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "death_ray/zero", rclcpp::QoS(10), std::bind(&DeathRayMotorControlNode::deathRayZeroCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "DeathRayMotorControlNode initialization complete.");
}

DeathRayMotorControlNode::~DeathRayMotorControlNode() {
    if (dir_line) {
        gpiod_line_set_value(dir_line, 0);
        gpiod_line_release(dir_line);
    }
    if (step_line) {
        gpiod_line_set_value(step_line, 0);
        gpiod_line_release(step_line);
    }
    if (chip) gpiod_chip_close(chip);
}

/**
 * Callback for the /death_ray/motor_commands subscriber.
 * Receives and decodes Float32 messages.
 * 
 * The sign of the command indicates the direction 
 * to rotate the dish.
 * NEGATIVE -> Counterclockwise
 * POSITIVE -> Clockwise
 * 
 * The magnitude of the command indicates the number of degrees 
 * to rotate the dish. This function decodes the command and 
 * transmits it to the GPIO pins.
 */
void DeathRayMotorControlNode::deathRayMotorCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    int steps;

    if (DEATH_RAY_CONTROL_MODE == DeathRayControlMode::ABS) {
        float cmd = msg->data;

        if (cmd > position) {
            gpiod_line_set_value(dir_line, STEPPER_CLOCKWISE_DIRECTION);
        }
        else if (cmd < position) {
            gpiod_line_set_value(dir_line, !STEPPER_CLOCKWISE_DIRECTION);
        }

        float degrees = std::abs(cmd - position);
        steps = std::round(degrees * DISH_PULSES_PER_DEGREE);

        position = cmd;
    }
    else { // Relative mode
        float cmd = msg->data;

        if (cmd > 0) {
            gpiod_line_set_value(dir_line, STEPPER_CLOCKWISE_DIRECTION);
        } else if (cmd < 0) {
            gpiod_line_set_value(dir_line, !STEPPER_CLOCKWISE_DIRECTION);
        }

        float degrees = std::abs(cmd);
        steps = std::round(degrees * DISH_PULSES_PER_DEGREE);
    }

    /**
     * rclcpp::ok on each loop iteration ensures that the loop stops
     * running if the node stops running
     * 
     * Without this, ctrl+C would not be registered until the loop finshed
     * running.
     */
    for (int i = 0; i < steps && rclcpp::ok(); i++) {
        gpiod_line_set_value(step_line, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(STEPPER_PULSE_DELAY_MS));
        gpiod_line_set_value(step_line, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(STEPPER_PULSE_DELAY_MS));
    }
}

/**
 * Callback for the death_ray/zero subscriber.
 * 
 * Sets the current tracked position to 0, for manual homing.
 */
void DeathRayMotorControlNode::deathRayZeroCallback(const std_msgs::msg::Empty::SharedPtr msg) {
    /**
     * Message content is empty: receiving any message on this topic indicates that the dish
     * should be zeroed.
     * 
     * There doesn't seem to be a way to define a subscriber callback without any parameters,
     * so this prevents compiler errors due to an unused parameter.
     */
    (void) msg;

    position = 0;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeathRayMotorControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
