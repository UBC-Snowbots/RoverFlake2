#include "comms_relay.h"
#include <algorithm>
#include <cmath>

ServoControlNode::ServoControlNode() : Node("servo_control_node") {
    chip_ = gpiod_chip_open(GPIO_CHIP_NAME);
    if (!chip_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open GPIO chip: %s", GPIO_CHIP_NAME);
        return;
    }

    servo1_line_ = gpiod_chip_get_line(chip_, SERVO1_GPIO_PIN);
    servo2_line_ = gpiod_chip_get_line(chip_, SERVO2_GPIO_PIN);

    if (!servo1_line_ || !servo2_line_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get one or more servo GPIO lines");
        if (chip_) {
            gpiod_chip_close(chip_);
            chip_ = nullptr;
        }
        return;
    }

    if (gpiod_line_request_output(servo1_line_, "comms_relay_servo1", 0) < 0 ||
        gpiod_line_request_output(servo2_line_, "comms_relay_servo2", 0) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to request servo GPIO lines as outputs");
        if (servo1_line_) {
            gpiod_line_release(servo1_line_);
            servo1_line_ = nullptr;
        }
        if (servo2_line_) {
            gpiod_line_release(servo2_line_);
            servo2_line_ = nullptr;
        }
        if (chip_) {
            gpiod_chip_close(chip_);
            chip_ = nullptr;
        }
        return;
    }

    gpiod_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "gpiod initialized successfully");

    // On power-up: immediately drive PWM to a known-safe starting state
    // (servo1 at MIN, servo2 in catch position) before anything else runs.
    setServoAngle(SERVO1_GPIO_PIN, SERVO1_MIN_ANGLE);
    setServoAngle(SERVO2_GPIO_PIN, SERVO2_CATCH_ANGLE);
    servo1_position_ = SERVO1_MIN_ANGLE;
    servo2_position_ = SERVO2_CATCH_ANGLE;
    state_ = RackState::CATCH;
    dwell_ticks_remaining_ = ENDPOINT_DWELL_MS / STATE_MACHINE_TICK_MS;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(64));
    servo1_position_pub_ = this->create_publisher<std_msgs::msg::Float32>("servo/servo1_position", qos);
    servo2_position_pub_ = this->create_publisher<std_msgs::msg::Float32>("servo/servo2_position", qos);
    state_pub_ = this->create_publisher<std_msgs::msg::String>("servo/rack_state", qos);

    state_machine_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(STATE_MACHINE_TICK_MS),
        std::bind(&ServoControlNode::tick, this));

    pwm_cycle_start_ = std::chrono::steady_clock::now();
    pwm_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&ServoControlNode::updatePwm, this));

    position_feedback_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(POSITION_FEEDBACK_PUBLISH_FREQUENCY_MS),
        std::bind(&ServoControlNode::publishPositions, this));

    RCLCPP_INFO(this->get_logger(), "ServoControlNode initialization complete. Starting in CATCH state.");
}

ServoControlNode::~ServoControlNode() {
    if (gpiod_ready_) {
        gpiod_line_set_value(servo1_line_, 0);
        gpiod_line_set_value(servo2_line_, 0);
    }

    if (servo1_line_) {
        gpiod_line_release(servo1_line_);
        servo1_line_ = nullptr;
    }
    if (servo2_line_) {
        gpiod_line_release(servo2_line_);
        servo2_line_ = nullptr;
    }
    if (chip_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }
}

/**
 * Maps a commanded angle in [-90, 90] degrees to a pulse width in
 * microseconds and updates the software PWM target pulse width.
 * Out-of-range values are clamped rather than rejected.
 */
void ServoControlNode::setServoAngle(int gpio_pin, float angle_deg) {
    float clamped = std::clamp(angle_deg, SERVO1_MIN_ANGLE, SERVO1_MAX_ANGLE);
    float t = (clamped - SERVO1_MIN_ANGLE) / (SERVO1_MAX_ANGLE - SERVO1_MIN_ANGLE);
    int pulse_us = SERVO_MIN_PULSE_US + static_cast<int>(t * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US));

    if (gpio_pin == SERVO1_GPIO_PIN) {
        servo1_pulse_us_ = pulse_us;
    } else if (gpio_pin == SERVO2_GPIO_PIN) {
        servo2_pulse_us_ = pulse_us;
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown servo GPIO pin: %d", gpio_pin);
    }
}

void ServoControlNode::updatePwm() {
    if (!gpiod_ready_ || !servo1_line_ || !servo2_line_) {
        return;
    }

    auto now = std::chrono::steady_clock::now();
    int elapsed_us = static_cast<int>(
        std::chrono::duration_cast<std::chrono::microseconds>(now - pwm_cycle_start_).count());

    if (elapsed_us >= SERVO_PWM_PERIOD_US) {
        pwm_cycle_start_ = now;
        elapsed_us = 0;
    }

    int servo1_level = (elapsed_us < servo1_pulse_us_) ? 1 : 0;
    int servo2_level = (elapsed_us < servo2_pulse_us_) ? 1 : 0;

    gpiod_line_set_value(servo1_line_, servo1_level);
    gpiod_line_set_value(servo2_line_, servo2_level);
}

const char* ServoControlNode::stateName(RackState s) {
    switch (s) {
        case RackState::CATCH:        return "CATCH";
        case RackState::SWEEP_TO_MAX: return "SWEEP_TO_MAX";
        case RackState::RELEASE:      return "RELEASE";
        case RackState::SWEEP_TO_MIN: return "SWEEP_TO_MIN";
        case RackState::DONE:         return "DONE";
    }
    return "UNKNOWN";
}

/**
 * Advances the state machine by one tick. Drives servo1 incrementally
 * during sweep states, snaps servo2 to catch/release at the appropriate
 * endpoint states, and stops the state machine after exactly one full
 * cycle (CATCH -> SWEEP_TO_MAX -> RELEASE -> SWEEP_TO_MIN -> DONE).
 */
void ServoControlNode::tick() {
    float step = SWEEP_DEG_PER_SEC * (STATE_MACHINE_TICK_MS / 1000.0f);

    switch (state_) {
        case RackState::CATCH:
            if (--dwell_ticks_remaining_ <= 0) {
                state_ = RackState::SWEEP_TO_MAX;
            }
            break;

        case RackState::SWEEP_TO_MAX:
            servo1_position_ += step;
            if (servo1_position_ >= SERVO1_MAX_ANGLE) {
                servo1_position_ = SERVO1_MAX_ANGLE;
                setServoAngle(SERVO1_GPIO_PIN, servo1_position_);
                state_ = RackState::RELEASE;
                setServoAngle(SERVO2_GPIO_PIN, SERVO2_RELEASE_ANGLE);
                servo2_position_ = SERVO2_RELEASE_ANGLE;
                dwell_ticks_remaining_ = ENDPOINT_DWELL_MS / STATE_MACHINE_TICK_MS;
                return;
            }
            setServoAngle(SERVO1_GPIO_PIN, servo1_position_);
            break;

        case RackState::RELEASE:
            if (--dwell_ticks_remaining_ <= 0) {
                state_ = RackState::SWEEP_TO_MIN;
            }
            break;

        case RackState::SWEEP_TO_MIN:
            servo1_position_ -= step;
            if (servo1_position_ <= SERVO1_MIN_ANGLE) {
                servo1_position_ = SERVO1_MIN_ANGLE;
                setServoAngle(SERVO1_GPIO_PIN, servo1_position_);
                state_ = RackState::CATCH;
                setServoAngle(SERVO2_GPIO_PIN, SERVO2_CATCH_ANGLE);
                servo2_position_ = SERVO2_CATCH_ANGLE;
                RCLCPP_INFO(this->get_logger(), "Cycle complete. Stopping state machine.");
                state_ = RackState::DONE;
                state_machine_timer_->cancel();
                return;
            }
            setServoAngle(SERVO1_GPIO_PIN, servo1_position_);
            break;

        case RackState::DONE:
            // Nothing to do; timer is cancelled so this shouldn't fire again.
            break;
    }
}

void ServoControlNode::publishPositions() {
    std_msgs::msg::Float32 msg1;
    msg1.data = servo1_position_;
    servo1_position_pub_->publish(msg1);

    std_msgs::msg::Float32 msg2;
    msg2.data = servo2_position_;
    servo2_position_pub_->publish(msg2);

    std_msgs::msg::String state_msg;
    state_msg.data = stateName(state_);
    state_pub_->publish(state_msg);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}