#include "servo.hpp"

#include <pigpiod_if2.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace {

// pigpio refuses servo pulse widths outside this range (0 means "off").
constexpr float PIGPIO_MIN_PULSE_US = 500.0f;
constexpr float PIGPIO_MAX_PULSE_US = 2500.0f;

void requireWithin(const ServoType& type, const char* field, float value, float lo, float hi) {
    if (!std::isfinite(value)) {
        throw std::runtime_error("servo type '" + type.name + "': " + field +
                                 " is not a finite number");
    }
    if (value < lo || value > hi) {
        throw std::runtime_error("servo type '" + type.name + "': " + field + " = " +
                                 std::to_string(std::lround(value)) + " us is outside [" +
                                 std::to_string(std::lround(lo)) + ", " +
                                 std::to_string(std::lround(hi)) + "]");
    }
}

}  // namespace

void ServoType::validate() const {
    requireWithin(*this, "min_us", min_us, PIGPIO_MIN_PULSE_US, PIGPIO_MAX_PULSE_US);
    requireWithin(*this, "max_us", max_us, PIGPIO_MIN_PULSE_US, PIGPIO_MAX_PULSE_US);
    if (min_us >= max_us) {
        throw std::runtime_error("servo type '" + name + "': min_us must be less than max_us");
    }
    requireWithin(*this, "open_us", open_us, min_us, max_us);
    requireWithin(*this, "close_us", close_us, min_us, max_us);
}

Servo::Servo(std::string name, int gpio_pin, ServoType type, int pi)
    : name_(std::move(name)), gpio_pin_(gpio_pin), type_(std::move(type)), pi_(pi) {}

bool Servo::setPulseUs(float pulse_us) {
    if (!std::isfinite(pulse_us)) {
        return false;
    }
    return send(std::clamp(pulse_us, type_.min_us, type_.max_us));
}

bool Servo::release() {
    if (pi_ < 0 || set_servo_pulsewidth(pi_, gpio_pin_, 0) != 0) {
        return false;
    }
    pulse_us_ = 0.0f;
    return true;
}

bool Servo::send(float pulse_us) {
    const auto rounded = static_cast<unsigned>(std::lround(pulse_us));
    if (pi_ < 0 || set_servo_pulsewidth(pi_, gpio_pin_, rounded) != 0) {
        return false;
    }
    pulse_us_ = pulse_us;
    return true;
}
