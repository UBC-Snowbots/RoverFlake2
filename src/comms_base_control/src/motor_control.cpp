#include "motor_control.h"

MotorControlNode::MotorControlNode() : Node("motor_control_node") {
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

    en_line = gpiod_chip_get_line(chip, EN_LINE_GPIO_PIN);
    if (!en_line) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get EN line on pin %d", EN_LINE_GPIO_PIN);
    } else {
        gpiod_line_request_output(en_line, "stepper_enable", 0);
        RCLCPP_INFO(this->get_logger(), "Successfully requested EN line on pin %d", EN_LINE_GPIO_PIN);
        
        gpiod_line_set_value(en_line, 1);
        RCLCPP_INFO(this->get_logger(), "Stepper driver ENABLED (Active-High). Coils energized.");
    }

    death_ray_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        "death_ray_commands", rclcpp::QoS(10), std::bind(&MotorControlNode::deathRayCommandCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to topic: 'death_ray_commands'");
    RCLCPP_INFO(this->get_logger(), "MotorControlNode initialization complete.");
}

MotorControlNode::~MotorControlNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down MotorControlNode, releasing GPIO lines...");
    if (en_line) {
        gpiod_line_set_value(en_line, 0);
        gpiod_line_release(en_line);
        RCLCPP_INFO(this->get_logger(), "Stepper driver disabled safely.");
    }
    if (dir_line) gpiod_line_release(dir_line);
    if (step_line) gpiod_line_release(step_line);
    if (chip) gpiod_chip_close(chip);
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
void MotorControlNode::deathRayCommandCallback(const std_msgs::msg::Int16::SharedPtr msg) {
    // Message received from topic will encode the direction in the sign and the num steps in the magnitude
    int stepper_cmd = msg->data;

    RCLCPP_INFO(this->get_logger(), "Received command: %d", stepper_cmd);

    // Drive the dir_line using the sign of the command
    if (stepper_cmd > 0) {
        RCLCPP_INFO(this->get_logger(), "Setting direction: CLOCKWISE (%d)", STEPPER_CLOCKWISE_DIRECTION);
        gpiod_line_set_value(dir_line, STEPPER_CLOCKWISE_DIRECTION);
    }
    else if (stepper_cmd < 0) {
        RCLCPP_INFO(this->get_logger(), "Setting direction: COUNTER-CLOCKWISE (%d)", !STEPPER_CLOCKWISE_DIRECTION);
        gpiod_line_set_value(dir_line, !STEPPER_CLOCKWISE_DIRECTION);
    }
    else {
        RCLCPP_WARN(this->get_logger(), "Received 0 steps connection command. Standing still.");
    }

    int steps = std::abs(stepper_cmd);
    RCLCPP_INFO(this->get_logger(), "Executing %d steps...", steps);

    // Drive the step_line using the magnitude of the command
    for (int i = 0; i < steps; i++) {
        gpiod_line_set_value(step_line, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(STEPPER_PULSE_DELAY_MS));
        gpiod_line_set_value(step_line, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(STEPPER_PULSE_DELAY_MS));
    }

    RCLCPP_INFO(this->get_logger(), "Finished executing %d steps.", steps);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
