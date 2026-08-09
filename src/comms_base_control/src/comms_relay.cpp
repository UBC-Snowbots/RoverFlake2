#include "comms_relay.h"
#include <algorithm>
#include <chrono>
#include <cmath>

using ServoCommand = rover_msgs::msg::ServoCommand;

ServoControlNode::ServoControlNode() : Node("servo_control_node") {
    default_hold_ms_ = static_cast<int>(this->declare_parameter<int>("default_hold_ms", 1000));

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

    // Subscribed even when pigpiod is unreachable, so commands are logged as
    // rejected instead of vanishing into a topic nobody listens on.
    servo_command_sub_ = this->create_subscription<ServoCommand>(
        "/servo_control_node/servo_command", 10,
        std::bind(&ServoControlNode::handleServoCommand, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "ServoControlNode initialization complete. Starting in CATCH state.");
}

ServoControlNode::~ServoControlNode() {
    if (pi_ >= 0) {
        set_servo_pulsewidth(pi_, SERVO_RP_GPIO_PIN, SERVO_RP_MAX_PWM);
        set_servo_pulsewidth(pi_, SERVO_CLAW_GPIO_PIN, 0);
        pigpio_stop(pi_);
    }
}

bool ServoControlNode::lookupServo(uint8_t servo, ServoSpec& spec) const {
    switch (servo) {
        case ServoCommand::SERVO_RP:
            spec = {"RP", SERVO_RP_GPIO_PIN, SERVO_RP_MIN_PWM, SERVO_RP_MAX_PWM};
            return true;
        case ServoCommand::SERVO_CLAW:
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

void ServoControlNode::handleServoCommand(const ServoCommand::SharedPtr msg) {
    if (pi_ < 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "servo_command ignored: not connected to pigpio daemon");
        return;
    }

    ServoSpec spec;
    if (!lookupServo(msg->servo, spec)) {
        RCLCPP_ERROR(this->get_logger(),
                     "servo_command ignored: unknown servo %u (expected 0=RP or 1=CLAW)",
                     msg->servo);
        return;
    }

    float pwm;
    switch (msg->mode) {
        case ServoCommand::MODE_LIMIT_MIN:
            pwm = spec.min_pwm;
            break;
        case ServoCommand::MODE_LIMIT_MAX:
            pwm = spec.max_pwm;
            break;
        case ServoCommand::MODE_PWM:
            if (!std::isfinite(msg->pwm_us)) {
                RCLCPP_ERROR(this->get_logger(),
                             "servo_command ignored: requested pulse width is not a finite number");
                return;
            }
            // Clamp rather than reject, so a slightly out-of-range command still
            // moves the servo to the nearest safe endpoint instead of doing nothing.
            pwm = std::clamp(msg->pwm_us, spec.min_pwm, spec.max_pwm);
            if (pwm != msg->pwm_us) {
                RCLCPP_WARN(this->get_logger(), "%s servo: %.0f us clamped to %.0f us", spec.name,
                            msg->pwm_us, pwm);
            }
            break;
        default:
            RCLCPP_ERROR(this->get_logger(),
                         "servo_command ignored: unknown mode %u (expected 0=MIN, 1=MAX, 2=PWM)",
                         msg->mode);
            return;
    }

    const uint16_t hold_ms =
        msg->hold_ms != 0 ? msg->hold_ms : static_cast<uint16_t>(default_hold_ms_);

    setServoPWM(spec.gpio_pin, pwm);
    RCLCPP_INFO(this->get_logger(), "%s servo -> %.0f us (holding %u ms)", spec.name, pwm, hold_ms);

    scheduleReturnToRest(spec, hold_ms);
}

void ServoControlNode::scheduleReturnToRest(const ServoSpec& spec, uint16_t hold_ms) {
    const int gpio_pin = spec.gpio_pin;
    const char* name = spec.name;
    const float rest_pwm = spec.min_pwm;

    // Assigning over the entry cancels whatever hold was already pending for
    // this servo, so the newest command owns the return.
    return_timers_[gpio_pin] = this->create_wall_timer(
        std::chrono::milliseconds(hold_ms), [this, gpio_pin, name, rest_pwm]() {
            // One-shot: stop the timer before doing the work so it cannot repeat.
            auto it = return_timers_.find(gpio_pin);
            if (it != return_timers_.end()) {
                it->second->cancel();
            }
            setServoPWM(gpio_pin, rest_pwm);
            RCLCPP_INFO(this->get_logger(), "%s servo returned to rest (%.0f us)", name, rest_pwm);
        });
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
