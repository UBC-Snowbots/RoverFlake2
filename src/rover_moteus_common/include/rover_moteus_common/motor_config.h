#pragma once

#include <vector>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

struct MotorConfig {
    // Motion profile limits written to the controller
    float max_acceleration = 3.0f;   // rev/s²  → servo.default_accel_limit
    float max_velocity     = 0.05f;  // rev/s   → servo.default_velocity_limit
                                     //           and servo.max_velocity

    // Software position limits (output revolutions)
    float position_min     = -1.0f;  // rev     → servopos.position_min
    float position_max     =  1.0f;  // rev     → servopos.position_max

    // How close to a limit before the driver logs a warning (not a hard stop)
    float position_warn_rev_padding = 0.02f;  // rev

    // Electrical safety limits
    float max_current_A    = 0.5f;   // A       → servo.max_current_A
    float max_voltage      = 26.0f;  // V       → servo.max_voltage
    float max_power_W      = 200.0f; // W       → servo.max_power_W

    // Position PID gains
    float kp = 50.0f;   //           → servo.pid_position.kp
    float ki =  0.0f;   //           → servo.pid_position.ki
    float kd =  0.0f;   //           → servo.pid_position.kd

    // Watchdog timeout — NaN means use firmware default (~0.1 s)
    float def_timeout = std::numeric_limits<float>::quiet_NaN(); // → servo.default_timeout_s

    // Gear reduction (informational only — firmware uses its own configured value)
    // Stored here so the HMI can display it in the Motor Config panel.
    float gear_reduction = 1.0f;

    // Datasheet identity (Maxon).  poles/kv feed calibration flags — these are
    // PER MOTOR: calibrating with the wrong pole count mis-commutates the
    // motor (low torque at max current, siren whine, angle-dependent gains).
    int         poles             = 16;    // pole count (pole pairs x 2)
    float       kv                = 265.0f;
    float       nominal_current_A = 1.0f;  // continuous thermal rating
    const char* part_no           = "?";

    // -------------------------------------------------------------------------
    // Helpers used by configureMotor()
    // -------------------------------------------------------------------------

    std::string format(float val) const {
        return std::isnan(val) ? "nan" : std::to_string(val);
    }

    // Returns (register_name, value_string) pairs sent via "conf set" on startup.
    std::vector<std::pair<std::string, std::string>> get_configs() const {
        return {
            {"servo.default_accel_limit",    format(max_acceleration)},
            {"servo.default_velocity_limit", format(max_velocity)},
            {"servo.max_velocity",           format(max_velocity)},
            {"servopos.position_min",        format(position_min)},
            {"servopos.position_max",        format(position_max)},
            {"servo.max_current_A",          format(max_current_A)},
            {"servo.pid_position.kp",        format(kp)},
            {"servo.pid_position.ki",        format(ki)},
            {"servo.pid_position.kd",        format(kd)},
            {"servo.max_voltage",            format(max_voltage)},
            {"servo.max_power_W",            format(max_power_W)},
            {"servo.default_timeout_s",      format(def_timeout)},
        };
    }
};