#pragma once

#include "motor_addressing.h"  // NUM_MOTORS

#include <cmath>
#include <limits>
#include <string>
#include <vector>

// =============================================================================
// Motor Configuration  (motor_config.h)
// =============================================================================
//
// Defines the per-motor servo parameters pushed to each moteus controller at
// startup via DiagnosticCommand ("conf set <register> <value>").
//
// WHEN TO CHANGE THESE:
//   - PID gains (kp/kd): if the arm oscillates or feels sluggish
//   - Current limits: protect motors from overheating / hardware limits
//   - Position limits: software travel limits to protect the arm geometry
//   - Velocity limits: cap maximum speed for safety
//
// HOW THEY ARE APPLIED:
//   1. On node startup, configureMotors() calls configureMotor() for each axis.
//   2. configureMotor() calls controller.DiagnosticCommand("conf set ...")
//      for every entry returned by MotorConfig::get_configs().
//   3. Settings are stored in the moteus RAM and take effect immediately.
//      They are NOT persisted to flash unless you separately run "conf write"
//      on the moteus tool — so they reset if the motor loses power.
//
// UNITS:
//   All position/velocity values are OUTPUT-SHAFT units (gear ratio applied
//   by firmware).  See motor_addressing.h for gear_reduction values.
// =============================================================================

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


// =============================================================================
// Per-axis configuration values — edit this function to tune the arm.
// Each index corresponds to ARM_JOINTS[i] in motor_addressing.h.
// =============================================================================

inline std::vector<MotorConfig> get_arm_configuration() {
    std::vector<MotorConfig> axes(NUM_MOTORS);

    // --- PID gains (hand-tuned per axis) ---
    //     Higher kp = stiffer.  Add kd to dampen oscillation.
    axes[0].kp = 180.0f;   axes[0].kd =  40.0f;   // Base
    axes[1].kp = 2100.0f;  axes[1].kd = 100.0f;   // Shoulder
    axes[2].kp = 4000.0f;  axes[2].kd = 750.0f;   // Elbow
    axes[3].kp =  50.0f;   axes[3].kd =   0.0f;   // Wrist Pitch
    axes[4].kp = 600.0f;   axes[4].kd = 100.0f;   // Wrist Roll
    axes[5].kp = 600.0f;   axes[5].kd = 100.0f;   // End Effector

    // --- Gear reductions (for display in HMI only — firmware holds the real value) ---
    axes[0].gear_reduction = 1.0f / 190.0f;
    axes[1].gear_reduction = 1.0f / 160.0f;
    axes[2].gear_reduction = 1.0f / 120.0f;
    axes[3].gear_reduction = 1.0f / 190.0f;
    axes[4].gear_reduction = 1.0f /  66.0f;
    axes[5].gear_reduction = 1.0f /  66.0f;

    // --- Current limits (A) ---
    axes[0].max_current_A = 1.0f;
    axes[1].max_current_A = 8.0f;
    axes[2].max_current_A = 5.5f;
    axes[3].max_current_A = 0.5f;
    axes[4].max_current_A = 2.5f;
    axes[5].max_current_A = 2.5f;

    // --- Software position limits (output-shaft revolutions) ---
    axes[0].position_min = -0.3f;     axes[0].position_max =   0.3f;
    axes[1].position_min = -0.47f;    axes[1].position_max =  -0.05f;
    axes[2].position_min =  0.01f;    axes[2].position_max =   0.4f;
    // axes[3]: uses default (-1.0, 1.0)
    axes[4].position_min = -999.01f;  axes[4].position_max = 999.4f;  // continuous
    axes[5].position_min = -999.01f;  axes[5].position_max = 999.4f;  // continuous

    return axes;
}
