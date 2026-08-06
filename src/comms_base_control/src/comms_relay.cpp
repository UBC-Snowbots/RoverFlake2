#include "comms_relay.h"
#include <algorithm>
#include <cmath>

ServoControlNode::ServoControlNode() : Node("servo_control_node") {
    gpioCfgInterfaces(PI_DISABLE_FIFO_IF | PI_DISABLE_SOCK_IF);
    if (gpioInitialise() < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize pigpio");
        return;
    }
    pigpio_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "pigpio initialized successfully");

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

    position_feedback_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(POSITION_FEEDBACK_PUBLISH_FREQUENCY_MS),
        std::bind(&ServoControlNode::publishPositions, this));

    RCLCPP_INFO(this->get_logger(), "ServoControlNode initialization complete. Starting in CATCH state.");
}

ServoControlNode::~ServoControlNode() {
    if (pigpio_ready_) {
        gpioServo(SERVO1_GPIO_PIN, 0);
        gpioServo(SERVO2_GPIO_PIN, 0);
        gpioTerminate();
    }
}

/**
 * Maps a commanded angle in [-90, 90] degrees to a pulse width in
 * microseconds and writes it to the given GPIO pin via pigpio.
 * Out-of-range values are clamped rather than rejected.
 */
void ServoControlNode::setServoAngle(int gpio_pin, float angle_deg) {
    float clamped = std::clamp(angle_deg, SERVO1_MIN_ANGLE, SERVO1_MAX_ANGLE);
    float t = (clamped - SERVO1_MIN_ANGLE) / (SERVO1_MAX_ANGLE - SERVO1_MIN_ANGLE);
    int pulse_us = SERVO_MIN_PULSE_US + static_cast<int>(t * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US));
    gpioServo(gpio_pin, pulse_us);
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