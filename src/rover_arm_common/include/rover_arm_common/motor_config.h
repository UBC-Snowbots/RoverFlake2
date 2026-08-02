#pragma once

#include "motor_addressing.h"  // NUM_MOTORS

#include <cmath>
#include <cstdint>
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
    std::vector<MotorConfig> motors(NUM_MOTORS);

    // --- Synced from controller flash 2026-08-02 (conf get readback) so the
    //     repo matches the bench tuning.  A7/EE was not replying; it keeps
    //     struct defaults until it can be read.

    // --- PID gains (hand-tuned per axis) ---
    //     Higher kp = stiffer.  Add kd to dampen oscillation.
    motors[0].kp =  4000.0f;  motors[0].kd =  600.0f;   // A1
    motors[1].kp = 30000.0f;  motors[1].kd = 6000.0f;   // A2
    motors[2].kp = 40000.0f;  motors[2].kd = 2000.0f;   // A3
    motors[3].kp =   550.0f;  motors[3].kd =   10.0f;   // A4
    motors[4].kp = 17000.0f;  motors[4].kd = 3500.0f;   // A5
    motors[5].kp = 17000.0f;  motors[5].kd = 3500.0f;   // A6

    // --- Gear reductions (for display in HMI only — firmware holds the real value) ---
    motors[0].gear_reduction = 1.0f / 190.0f;
    motors[1].gear_reduction = 1.0f / 160.0f;
    motors[2].gear_reduction = 1.0f / 120.0f;
    motors[3].gear_reduction = 1.0f / 190.0f;
    motors[4].gear_reduction = 1.0f /  66.0f;
    motors[5].gear_reduction = 1.0f /  66.0f;

    // --- Current limits (A) ---
    motors[0].max_current_A =  3.0f;   // raised from 2.0 on bench 2026-08-02: A1 stalled at 2 A
    motors[1].max_current_A = 14.0f;
    motors[2].max_current_A = 14.0f;
    motors[3].max_current_A =  0.3f;
    motors[4].max_current_A =  2.0f;
    motors[5].max_current_A =  2.0f;

    // --- Voltage / power limits ---
    const float fnan = std::numeric_limits<float>::quiet_NaN();
    motors[0].max_voltage = 30.0f;  motors[0].max_power_W = 200.0f;
    motors[1].max_voltage = 30.0f;  motors[1].max_power_W = fnan;
    motors[2].max_voltage = 32.0f;  motors[2].max_power_W = 250.0f;
    motors[3].max_voltage = 24.0f;  motors[3].max_power_W = 30.0f;
    motors[4].max_voltage = 30.0f;  motors[4].max_power_W = fnan;
    motors[5].max_voltage = 30.0f;  motors[5].max_power_W = fnan;

    // --- Motion profile ---
    motors[0].max_acceleration = 1.0f;  motors[0].max_velocity = 0.15f;
    motors[1].max_acceleration = 0.5f;  motors[1].max_velocity = 0.03f;
    motors[2].max_acceleration = 0.5f;  motors[2].max_velocity = 0.05f;
    motors[3].max_acceleration = 0.5f;  motors[3].max_velocity = 0.10f;
    motors[4].max_acceleration = 2.0f;  motors[4].max_velocity = 0.15f;
    motors[5].max_acceleration = 2.0f;  motors[5].max_velocity = 0.15f;

    // --- Position limits (output-shaft revolutions) ---
    //     Flash has servopos unbounded (nan) everywhere except A4 — adopted
    //     as-is; homing/soft-stop bounds live in AxisConfig, not here.
    for (int i : {0, 1, 2, 4, 5}) {
        motors[i].position_min = fnan;
        motors[i].position_max = fnan;
    }
    motors[3].position_min = -0.5f;  motors[3].position_max = 0.5f;
    motors[3].def_timeout  = 0.1f;

    return motors;
}


// Axis Configs
#define BANDAID_AXIS_5_MAX_POSITION_OUTPUT_REVS 0.5
#define BANDAID_AXIS_1_MAX_POSITION_OUTPUT_REVS 0.35

namespace AxisConfig {
    // In Revolutions
    float max_position_rev[NUM_AXES] = {
        /*AXIS 1 */   0.4,
        /*AXIS 2 */   0.5,
        /*AXIS 3 */   0.5,
        /*AXIS 4 */   0.5,
        /*AXIS 5 */   0.5,
        /*AXIS 6 */   0.5,
        /*AXIS EE */  0.5 };
         
    float min_position_rev[NUM_AXES] = {0, 0, 0, 0, 0, 0, 0};

    float homing_speed_revps[NUM_AXES] = { // Rev/s Direction Independent. 
        /*AXIS 1 */   0.01,
        /*AXIS 2 */   0.01,
        /*AXIS 3 */   0.01,
        /*AXIS 4 */   0.01,
        /*AXIS 5 */   0.01,
        /*AXIS 6 */   0.01,
        /*AXIS EE */  0.01 };

    int homing_direction[NUM_AXES] = { // Rev/s Direction Independent. 
        /*AXIS 1 */   -1,
        /*AXIS 2 */   -1,
        /*AXIS 3 */   -1,
        /*AXIS 4 */   -1,
        /*AXIS 5 */   1,
        /*AXIS 6 */   -1,
        /*AXIS EE */  -1 };

    float max_running_speed[NUM_AXES] = { // Rev/s Direction Independent. 
        /*AXIS 1 */   0.1,
        /*AXIS 2 */   0.1,
        /*AXIS 3 */   0.1,
        /*AXIS 4 */   0.1,
        /*AXIS 5 */   0.1,
        /*AXIS 6 */   0.1,
        /*AXIS EE */  0.5 };

    float max_running_accel[NUM_AXES] = { // Rev/s Direction Independent. 
        /*AXIS 1 */   0.5,
        /*AXIS 2 */   0.5,
        /*AXIS 3 */   0.5,
        /*AXIS 4 */   0.5,
        /*AXIS 5 */   0.5,
        /*AXIS 6 */   0.5,
        /*AXIS EE */  0.5 };
    
    float idle_position[NUM_AXES] = { // Rev/s Direction Independent.
        /*AXIS 1 */   0.1,
        /*AXIS 2 */   0.1,
        /*AXIS 3 */   0.1,
        /*AXIS 4 */   0.1,
        /*AXIS 5 */   -0.1,
        /*AXIS 6 */   0.1,
        /*AXIS EE */  0.1 };

    // Limit switches — AUX2 digital input, see setup_limit_switches.py.
    // mask: aux2 GPIO status bitfield, bit0 = aux2.pins.0 (ABS connector pin 2).
    // inverted=false: NC switch to GND (hard 2k pullup) -> pin HIGH = pressed.
    bool has_limit_switch[NUM_AXES] = {
        /*AXIS 1 */   true,
        /*AXIS 2 */   true,
        /*AXIS 3 */   true,
        /*AXIS 4 */   true,
        /*AXIS 5 */   false,
        /*AXIS 6 */   false,
        /*AXIS EE */  false };

    // Per-axis, measured with DEBUG_LIMIT_SWITCH_RAW_REPLY (2026-08-02).
    // All four: pressed = HIGH; only the pin differs (A1/A3 pin 0, A2/A4 pin 1).
    // A1's first capture was misread as inverted — the arm was resting on its
    // switch, so the observed transitions were releases, not presses.
    // A3's pin has no pull; if it flakes: conf set aux2.pins.0.pull 2 on motor 3.
    uint8_t limit_switch_mask[NUM_AXES]     = {1, 2, 1, 2, 0, 0, 0};
    bool    limit_switch_inverted[NUM_AXES] = {false, false, false, false, false, false, false};

};
