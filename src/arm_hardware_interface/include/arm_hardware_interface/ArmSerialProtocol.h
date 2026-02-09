#pragma once

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#define CONTROL_RATE 60.0
#define COMM_POLL_RATE 1000.0 // idea is to poll serial faster than arm can send messages. We don't wan't to miss any messages.

#define HOME_CMD 'h'
#define HOME_ALL_ID 50

#define ABS_POS_CMD 'P'
#define COMM_CMD 'C'
#define ABS_VEL_CMD 'V'

#define TEST_LIMITS_CMD 't'

#define EE_INDEX 6

// use one array with axis idx for motor config

// conf set servo.default_accel_limit nan
// conf set servo.default_velocity_limit nan
// Command: d rezero (or d rezero <position>)
// Note: This will change where the motor thinks "0" is.
// Check servo.default_timeout_s. If this motor has a much higher timeout than the other, it might be sitting in a "stale" state before
// accepting the next command.
// d exact 0
// ^- zero the position
// d zero¶
// Enter the zero velocity state. A zero velocity is commanded regardless of position.

struct MotorConfig {
  // ---------------------------------------------------------
  // 1. Motion Limits (Trajectory & Hard Limits)
  // ---------------------------------------------------------
  // Maximum acceleration for trajectory generation (revolutions/s^2)
  // Moteus Config: servo.default_accel_limit
  float max_acceleration = 20.0f;

  // Maximum velocity limit (revolutions/s)
  // Moteus Config: servo.velocity_limit (soft) or servo.max_velocity (hard limit)
  float max_velocity = 10.0f;

  // Position Limits (revolutions)
  // Moteus Config: servopos.position_min / servopos.position_max
  float position_min = -10.0f;
  float position_max = 10.0f;

  // ---------------------------------------------------------
  // 2. Torque / Current Limits
  // ---------------------------------------------------------
  // Max Torque is primarily limited by Phase Current in moteus.
  // Torque (Nm) ~= Current (A) * Torque_Constant (Nm/A)
  // Moteus Config: servo.max_current_A
  float max_current_A = 20.0f; // Used to enforce Max Torque

  // ---------------------------------------------------------
  // 3. PID Tuning Parameters
  // ---------------------------------------------------------
  // Moteus Config: servo.pid_position.kp, ki, kd
  float kp = 50.0f;
  float ki = 0.0f;
  float kd = 1.0f;

  // ---------------------------------------------------------
  // 4. Safety Limits (Voltage & Power)
  // ---------------------------------------------------------
  // Maximum input voltage allowed before faulting (Volts)
  // Moteus Config: servo.max_voltage
  float max_voltage = 48.0f;

  // Optional: Max Power limit to prevent overheating (Watts)
  // Moteus Config: servo.max_power_W
  float max_power_W = 450.0f;

  // Timing parameter
  // Moteus Config: servo.default_timeout_s
  float def_timeout = std::numeric_limits<float>::quiet_NaN();

  // others maybe to use:
  //  servo.default_timeout_s
  //  servo.timeout_mode
  //  servo.flux_brake_min_voltage
  //  motor_position.rotor_to_output_ratio
  //  motor_position.output.offset
  //  servo.derate_temperature
  //  servo.enable_motor_temperature
  //  servo.max_position_slip

  // Helper to ensure NaN is stringified as "nan" for Moteus compatibility
  std::string format(float val) const { return std::isnan(val) ? "nan" : std::to_string(val); }

  // ---------------------------------------------------------
  // Helper function to generate command pairs for moteus_tool or API
  // ---------------------------------------------------------
  std::vector<std::pair<std::string, std::string>> get_configs() const {
    return {{"servo.default_accel_limit", format(max_acceleration)},
            {"servo.default_velocity_limit", format(max_velocity)},
            {"servo.max_velocity", format(max_velocity)}, // Set hard limit same as trajectory
            {"servopos.position_min", format(position_min)},
            {"servopos.position_max", format(position_max)},
            {"servo.max_current_A", format(max_current_A)},
            {"servo.pid_position.kp", format(kp)},
            {"servo.pid_position.ki", format(ki)},
            {"servo.pid_position.kd", format(kd)},
            {"servo.max_voltage", format(max_voltage)},
            {"servo.max_power_W", format(max_power_W)},
            {"servo.default_timeout_s", format(def_timeout)}};
  }
};

// Function to generate the configuration for all axes
inline std::vector<MotorConfig> get_arm_configuration() {
  // Initialize 6 axes (Defaults match Axis 1, 2, 4, 5)
  std::vector<MotorConfig> axes(6);

  // --- Axis 3 Customization ---
  // gear reduction
  axes[2].max_acceleration = std::numeric_limits<float>::quiet_NaN();
  axes[2].max_velocity = std::numeric_limits<float>::quiet_NaN();
  axes[2].position_min = std::numeric_limits<float>::quiet_NaN();
  axes[2].position_max = std::numeric_limits<float>::quiet_NaN();
  axes[2].def_timeout = std::numeric_limits<float>::quiet_NaN();

  // --- Axis 6 Customization ---
  // gear reduction
  axes[5].position_min = std::numeric_limits<float>::quiet_NaN();
  axes[5].position_max = std::numeric_limits<float>::quiet_NaN();
  axes[5].def_timeout = std::numeric_limits<float>::quiet_NaN();

  // Note: Other axes (1, 2, 4, 5) use the default struct values
  return axes;
}

// TODO implement arm abort

//? Limit switch feedback looks like:
//? sprintf(tmpmsg, "Limit Switch %d, is %d.  \n\r\0", i + 1, get_gpio(axes[i].LIMIT_PIN[0], axes[i].LIMIT_PIN[1]));
