#include "comms_relay.h"
#include <algorithm>
#include <cmath>

using SetServoLimit = rover_msgs::srv::SetServoLimit;
using SetServoPwm = rover_msgs::srv::SetServoPwm;

ServoControlNode::ServoControlNode() : Node("servo_control_node") {
    // Connect to an already-running pigpiod rather than driving the hardware
    // directly, so this node does not need root. Passing nulls means
    // localhost:8888 unless PIGPIO_ADDR / PIGPIO_PORT say otherwise.
    pi_ = pigpio_start(nullptr, nullptr);
    if (pi_ < 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to connect to pigpio daemon (%d); is pigpiod running?", pi_);
    } else {
        RCLCPP_INFO(this->get_logger(), "Connected to pigpio daemon (handle %d)", pi_);

        setServoPWM(SERVO_RP_GPIO_PIN, SERVO_RP_MIN_PWM);
        setServoPWM(SERVO_CLAW_GPIO_PIN, SERVO_CLAW_MIN_PWM);
    }

    // Advertised even when pigpiod is unreachable, so callers get an explicit
    // failure response instead of a service that never appears.
    set_servo_limit_srv_ = this->create_service<SetServoLimit>(
        "~/set_servo_limit",
        std::bind(&ServoControlNode::handleSetServoLimit, this,
                  std::placeholders::_1, std::placeholders::_2));
    set_servo_pwm_srv_ = this->create_service<SetServoPwm>(
        "~/set_servo_pwm",
        std::bind(&ServoControlNode::handleSetServoPwm, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "ServoControlNode initialization complete. Starting in CATCH state.");
}

ServoControlNode::~ServoControlNode() {
    if (pi_ >= 0) {
        set_servo_pulsewidth(pi_, SERVO_RP_GPIO_PIN, 0);
        set_servo_pulsewidth(pi_, SERVO_CLAW_GPIO_PIN, 0);
        pigpio_stop(pi_);
    }
}

bool ServoControlNode::lookupServo(uint8_t servo, ServoSpec& spec) const {
    switch (servo) {
        case SetServoLimit::SERVO_RP:
            spec = {"RP", SERVO_RP_GPIO_PIN, SERVO_RP_MIN_PWM, SERVO_RP_MAX_PWM};
            return true;
        case SetServoLimit::SERVO_CLAW:
            spec = {"CLAW", SERVO_CLAW_GPIO_PIN, SERVO_CLAW_MIN_PWM, SERVO_CLAW_MAX_PWM};
            return true;
        default:
            return false;
    }
}

void ServoControlNode::setServoPWM(int gpio_pin, float PWM) {
    if (pi_ < 0) {
        return;
    }
    set_servo_pulsewidth(pi_, gpio_pin, PWM);
}

void ServoControlNode::handleSetServoLimit(
    const std::shared_ptr<SetServoLimit::Request> request,
    std::shared_ptr<SetServoLimit::Response> response) {
    response->success = false;
    response->pwm_us = 0.0f;

    if (pi_ < 0) {
        response->message = "not connected to pigpio daemon";
        RCLCPP_ERROR(this->get_logger(), "set_servo_limit rejected: %s", response->message.c_str());
        return;
    }

    ServoSpec spec;
    if (!lookupServo(request->servo, spec)) {
        response->message = "unknown servo " + std::to_string(request->servo) +
                            " (expected 0=RP or 1=CLAW)";
        RCLCPP_ERROR(this->get_logger(), "set_servo_limit rejected: %s", response->message.c_str());
        return;
    }

    float pwm;
    switch (request->limit) {
        case SetServoLimit::LIMIT_MIN:
            pwm = spec.min_pwm;
            break;
        case SetServoLimit::LIMIT_MAX:
            pwm = spec.max_pwm;
            break;
        default:
            response->message = "unknown limit " + std::to_string(request->limit) +
                                " (expected 0=MIN or 1=MAX)";
            RCLCPP_ERROR(this->get_logger(), "set_servo_limit rejected: %s", response->message.c_str());
            return;
    }

    setServoPWM(spec.gpio_pin, pwm);
    response->success = true;
    response->pwm_us = pwm;
    response->message = std::string(spec.name) +
                        (request->limit == SetServoLimit::LIMIT_MAX ? " at MAX" : " at MIN");
    RCLCPP_INFO(this->get_logger(), "%s servo -> %s (%.0f us)", spec.name,
                request->limit == SetServoLimit::LIMIT_MAX ? "MAX" : "MIN", pwm);
}

void ServoControlNode::handleSetServoPwm(
    const std::shared_ptr<SetServoPwm::Request> request,
    std::shared_ptr<SetServoPwm::Response> response) {
    response->success = false;
    response->pwm_us = 0.0f;

    if (pi_ < 0) {
        response->message = "not connected to pigpio daemon";
        RCLCPP_ERROR(this->get_logger(), "set_servo_pwm rejected: %s", response->message.c_str());
        return;
    }

    ServoSpec spec;
    if (!lookupServo(request->servo, spec)) {
        response->message = "unknown servo " + std::to_string(request->servo) +
                            " (expected 0=RP or 1=CLAW)";
        RCLCPP_ERROR(this->get_logger(), "set_servo_pwm rejected: %s", response->message.c_str());
        return;
    }

    if (!std::isfinite(request->pwm_us)) {
        response->message = "requested pulse width is not a finite number";
        RCLCPP_ERROR(this->get_logger(), "set_servo_pwm rejected: %s", response->message.c_str());
        return;
    }

    // Clamp rather than reject, so a slightly out-of-range command still moves
    // the servo to the nearest safe endpoint instead of doing nothing.
    const float pwm = std::clamp(request->pwm_us, spec.min_pwm, spec.max_pwm);

    setServoPWM(spec.gpio_pin, pwm);
    response->success = true;
    response->pwm_us = pwm;
    if (pwm != request->pwm_us) {
        response->message = "clamped to [" + std::to_string(static_cast<int>(spec.min_pwm)) + ", " +
                            std::to_string(static_cast<int>(spec.max_pwm)) + "] us";
        RCLCPP_WARN(this->get_logger(), "%s servo: %.0f us clamped to %.0f us", spec.name,
                    request->pwm_us, pwm);
    } else {
        response->message = std::string(spec.name) + " at requested pulse width";
        RCLCPP_INFO(this->get_logger(), "%s servo -> %.0f us", spec.name, pwm);
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
