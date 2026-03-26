#pragma once

#include <cmath>
#include <limits>
#include <string>
#include <vector>

constexpr int NUM_MOTORS = 6;

struct MotorConfig {
    float max_acceleration = 3.0f;        // rev/s^2  (servo.default_accel_limit)
    float max_velocity = 0.05f;           // rev/s    (servo.default_velocity_limit + servo.max_velocity)
    float position_min = -1.0f;           // rev      (servopos.position_min)
    float position_max = 1.0f;            // rev      (servopos.position_max)
    float position_warn_rev_padding = 0.02f;  // alert when within this of limits
    float max_current_A = 0.5f;           // A        (servo.max_current_A)
    float kp = 50.0f;                     //          (servo.pid_position.kp)
    float ki = 0.0f;                      //          (servo.pid_position.ki)
    float kd = 0.0f;                      //          (servo.pid_position.kd)
    float max_voltage = 26.0f;            // V        (servo.max_voltage)
    float max_power_W = 200.0f;           // W        (servo.max_power_W)
    float def_timeout = std::numeric_limits<float>::quiet_NaN();  // (servo.default_timeout_s)
    float gear_reduction = 1.0f;

    std::string format(float val) const {
        return std::isnan(val) ? "nan" : std::to_string(val);
    }

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

inline std::vector<MotorConfig> get_arm_configuration() {
    std::vector<MotorConfig> axes(NUM_MOTORS);

    // --- PID tuning (per-motor, hand-tuned) ---
    axes[0].kp = 180.0f;   axes[0].kd = 40.0f;    // Base
    axes[1].kp = 2100.0f;  axes[1].kd = 100.0f;   // Shoulder
    axes[2].kp = 4000.0f;  axes[2].kd = 750.0f;   // Elbow
    axes[3].kp = 50.0f;    axes[3].kd = 0.0f;     // Wrist Pitch
    axes[4].kp = 600.0f;   axes[4].kd = 100.0f;   // Wrist Roll
    axes[5].kp = 600.0f;   axes[5].kd = 100.0f;   // End Effector

    // --- Gear reductions ---
    axes[0].gear_reduction = 1.0f / 190.0f;
    axes[1].gear_reduction = 1.0f / 160.0f;
    axes[2].gear_reduction = 1.0f / 120.0f;
    axes[3].gear_reduction = 1.0f / 190.0f;
    axes[4].gear_reduction = 1.0f / 66.0f;
    axes[5].gear_reduction = 1.0f / 66.0f;

    // --- Current limits ---
    axes[0].max_current_A = 1.0f;
    axes[1].max_current_A = 8.0f;
    axes[2].max_current_A = 5.5f;
    axes[3].max_current_A = 0.5f;
    axes[4].max_current_A = 1.0f;
    axes[5].max_current_A = 1.0f;

    // --- Position limits (revolutions) ---
    axes[0].position_min = -0.3f;    axes[0].position_max = 0.3f;
    axes[1].position_min = -0.47f;   axes[1].position_max = -0.05f;
    axes[2].position_min = 0.01f;    axes[2].position_max = 0.4f;
    // axes[3] uses defaults (-1.0, 1.0)
    axes[4].position_min = -999.01f; axes[4].position_max = 999.4f;
    axes[5].position_min = -999.01f; axes[5].position_max = 999.4f;

    return axes;
}
