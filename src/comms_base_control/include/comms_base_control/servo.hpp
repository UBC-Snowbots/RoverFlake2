#pragma once
#include <string>

// Calibration of one servo model, loaded from config/servo_types/<name>.yaml.
struct ServoType {
    std::string name;
    float min_us = 0.0f;   // hard travel limits; never exceeded
    float max_us = 0.0f;
    float open_us = 0.0f;
    float close_us = 0.0f;

    // Throws std::runtime_error if the limits are inconsistent or outside what
    // pigpio accepts (500-2500 us).
    void validate() const;
};

// One physical servo: a ServoType bound to a GPIO pin, driven through pigpiod.
// Every method returns false if pigpiod refused (or pi is not connected).
class Servo {
public:
    // pi is the handle from pigpio_start; the caller owns the connection.
    Servo(std::string name, int gpio_pin, ServoType type, int pi);

    bool open() { return send(type_.open_us); }
    bool close() { return send(type_.close_us); }
    bool goToMin() { return send(type_.min_us); }
    bool goToMax() { return send(type_.max_us); }

    // Clamped to [min_us, max_us]; non-finite values are refused.
    bool setPulseUs(float pulse_us);

    // Stops the pulses so the servo goes limp.
    bool release();

    const std::string& name() const { return name_; }
    const ServoType& type() const { return type_; }

    // Last pulse width sent; 0 when released or never driven.
    float pulseUs() const { return pulse_us_; }

private:
    bool send(float pulse_us);

    std::string name_;
    int gpio_pin_;
    ServoType type_;
    int pi_;
    float pulse_us_ = 0.0f;
};
