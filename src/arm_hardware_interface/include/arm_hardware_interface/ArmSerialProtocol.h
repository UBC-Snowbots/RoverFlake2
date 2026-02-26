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

#define AXIS_1_INDEX 0
#define AXIS_2_INDEX 1
#define AXIS_3_INDEX 2
#define AXIS_4_INDEX 3
#define AXIS_5_INDEX 4
#define AXIS_6_INDEX 5
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
  float max_acceleration = 3.0f;

  // Maximum velocity limit (revolutions/s)
  // Moteus Config: servo.velocity_limit (soft) or servo.max_velocity (hard limit)
  float max_velocity = 0.05f;

  // Position Limits (revolutions)
  // Moteus Config: servopos.position_min / servopos.position_max
  float position_min = -1.0f;
  float position_max = 1.0f;

  // ---------------------------------------------------------
  // 2. Torque / Current Limits
  // ---------------------------------------------------------
  // Max Torque is primarily limited by Phase Current in moteus.
  // Torque (Nm) ~= Current (A) * Torque_Constant (Nm/A)
  // Moteus Config: servo.max_current_A
  float max_current_A = 0.5f; // Used to enforce Max Torque

  // ---------------------------------------------------------
  // 3. PID Tuning Parameters
  // ---------------------------------------------------------
  // Moteus Config: servo.pid_position.kp, ki, kd
  float kp = 50.0f;
  float ki = 0.0f;
  float kd = 0.0f;

  // ---------------------------------------------------------
  // 4. Safety Limits (Voltage & Power)
  // ---------------------------------------------------------
  // Maximum input voltage allowed before faulting (Volts)
  // Moteus Config: servo.max_voltage
  float max_voltage = 26.0f;

  // Optional: Max Power limit to prevent overheating (Watts)
  // Moteus Config: servo.max_power_W
  float max_power_W = 200.0f;

  // Timing parameter
  // Moteus Config: servo.default_timeout_s
  float def_timeout = std::numeric_limits<float>::quiet_NaN();

  float gear_red = 1;
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

  // PID
  axes[AXIS_1_INDEX].kp = 180.0; // SET - WORKING with p 180, d 40 ( 5 min spent tuning )
  axes[AXIS_1_INDEX].kd = 40.0;

  axes[AXIS_2_INDEX].kp = 2100.0; // SET - WORKING with p 2100 d 100 (2 min spent tuning)
  axes[AXIS_2_INDEX].kd = 100.0;

  axes[AXIS_3_INDEX].kp = 4000.0; // SET - working okay with p 4000, d 750 ( 5 min spent tuning )
  axes[AXIS_3_INDEX].kd = 750.0;

  axes[AXIS_4_INDEX].kp = 50.0;
  axes[AXIS_4_INDEX].kd = 0.0;
  
  axes[AXIS_5_INDEX].kp = 50.0;
  axes[AXIS_5_INDEX].kd = 0.0;
  
  axes[AXIS_6_INDEX].kp = 50.0;
  axes[AXIS_6_INDEX].kd = 0.0;


  // GEAR REDUCTION
  axes[AXIS_1_INDEX].gear_red = (1.0f/190.0f);
  axes[AXIS_2_INDEX].gear_red = (1.0f/160.0f);
  axes[AXIS_3_INDEX].gear_red = (1.0f/120.0f);
  axes[AXIS_4_INDEX].gear_red = (1.0f/190.0f);
  axes[AXIS_5_INDEX].gear_red = (1.0f/66.0f);
  axes[AXIS_6_INDEX].gear_red = (1.0f/66.0f);
  // EE coming soon
  // axes[EE_INDEX].gear_red = (1.0f/190.0f);


  // CURRENT LIMITS
  axes[AXIS_1_INDEX].max_current_A = 1.5f;
  axes[AXIS_2_INDEX].max_current_A = 5.0f; 
  axes[AXIS_3_INDEX].max_current_A = 2.0f; 
  axes[AXIS_4_INDEX].max_current_A = 0.5f; 
  axes[AXIS_5_INDEX].max_current_A = 0.5f;
  axes[AXIS_6_INDEX].max_current_A = 0.5f;
  
  axes[AXIS_1_INDEX].position_min = -0.15;
  axes[AXIS_1_INDEX].position_max = 0.15;

  axes[AXIS_2_INDEX].position_min = -0.18;
  axes[AXIS_2_INDEX].position_max = 0.15;

  axes[AXIS_3_INDEX].position_min = -0.2;
  axes[AXIS_3_INDEX].position_max = 0.2;

  // Note: Other axes (1, 2, 4, 5) use the default struct values
  return axes;
}

// TODO implement arm abort

//? Limit switch feedback looks like:
//? sprintf(tmpmsg, "Limit Switch %d, is %d.  \n\r\0", i + 1, get_gpio(axes[i].LIMIT_PIN[0], axes[i].LIMIT_PIN[1]));
