#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include <pigpio.h>

#define SERVO1_GPIO_PIN 17   // rack-and-pinion drive servo
#define SERVO2_GPIO_PIN 27   // catch/release latch servo

#define SERVO1_MIN_ANGLE -90.0f
#define SERVO1_MAX_ANGLE  90.0f

// Tune these to your specific servos' datasheets (microseconds)
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500

// Servo2 fixed positions, in the same -90..90 angle space as servo1
#define SERVO2_CATCH_ANGLE   -45.0f
#define SERVO2_RELEASE_ANGLE  45.0f

// Sweep behavior for servo1
#define SWEEP_DEG_PER_SEC 30.0f
#define STATE_MACHINE_TICK_MS 20   // 50Hz update loop

// How long to sit at each endpoint before reversing (lets servo2 settle)
#define ENDPOINT_DWELL_MS 300

#define POSITION_FEEDBACK_PUBLISH_FREQUENCY_MS 200

/**
 * State cycle (runs exactly once per node launch, then stops):
 * CATCH -> SWEEP_TO_MAX -> RELEASE -> SWEEP_TO_MIN -> DONE
 */
enum class RackState {
    CATCH,          // holding at MIN, servo2 catch
    SWEEP_TO_MAX,   // servo1 moving MIN -> MAX
    RELEASE,        // holding at MAX, servo2 release
    SWEEP_TO_MIN,   // servo1 moving MAX -> MIN
    DONE            // cycle complete, servo1 back at MIN holding, timer stopped
};

/**
 * @brief ServoControlNode drives a rack-and-pinion servo (servo1) through a
 * single -90 -> 90 -> -90 sweep, and a catch/release latch servo (servo2)
 * that automatically triggers at each sweep endpoint. Runs once per node
 * launch; there is no HMI, no repeating loop, and no external topic
 * trigger — the state machine starts as soon as the node comes up.
 */
class ServoControlNode : public rclcpp::Node {
public:
    ServoControlNode();
    ~ServoControlNode();

private:
    void tick();
    void setServoAngle(int gpio_pin, float angle_deg);
    void publishPositions();
    const char* stateName(RackState s);

    rclcpp::TimerBase::SharedPtr state_machine_timer_;
    rclcpp::TimerBase::SharedPtr position_feedback_timer_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr servo1_position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr servo2_position_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

    RackState state_ = RackState::CATCH;
    float servo1_position_ = SERVO1_MIN_ANGLE;
    float servo2_position_ = SERVO2_CATCH_ANGLE;

    // Dwell tracking at endpoints, in state-machine ticks
    int dwell_ticks_remaining_ = 0;

    bool pigpio_ready_ = false;
};