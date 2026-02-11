#pragma once

#include <cmath>
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

struct MotorConfigAxis1 {
  // ---------------------------------------------------------
  // 1. Motion Limits (Trajectory & Hard Limits)
  // ---------------------------------------------------------
  // Maximum acceleration for trajectory generation (revolutions/s^2)
  // Moteus Config: servo.default_accel_limit
  float max_acceleration = 0.5f;

  // Maximum velocity limit (revolutions/s)
  // Moteus Config: servo.velocity_limit (soft) or servo.max_velocity (hard limit)
  float max_velocity = 1.0f;

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
  float max_current_A = 1.0f; // Used to enforce Max Torque

  // ---------------------------------------------------------
  // 3. PID Tuning Parameters
  // ---------------------------------------------------------
  // Moteus Config: servo.pid_position.kp, ki, kd
  float kp = 10.0f;
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
  float max_power_W = 450.0f;
  float def_timeout = std::numeric_limits<double>::quiet_NaN();


  // others maybe to use:
  //  servo.default_timeout_s
  //  servo.timeout_mode
  //  servo.flux_brake_min_voltage
  //  motor_position.rotor_to_output_ratio
  //  motor_position.output.offset
  //  servo.derate_temperature
  //  servo.enable_motor_temperature
  //  servo.max_position_slip

  // ---------------------------------------------------------
  // Helper function to generate command pairs for moteus_tool or API
  // ---------------------------------------------------------
  std::vector<std::pair<std::string, std::string>> get_configs() const {
    return {{"servo.default_accel_limit", std::to_string(max_acceleration)},
            {"servo.default_velocity_limit", std::to_string(max_velocity)},
            {"servo.max_velocity", std::to_string(max_velocity)}, // Set hard limit same as trajectory
            {"servopos.position_min", std::to_string(position_min)},
            {"servopos.position_max", std::to_string(position_max)},
            {"servo.max_current_A", std::to_string(max_current_A)},
            {"servo.pid_position.kp", std::to_string(kp)},
            {"servo.pid_position.ki", std::to_string(ki)},
            {"servo.pid_position.kd", std::to_string(kd)},
            {"servo.max_voltage", std::to_string(max_voltage)},
            {"servo.default_timeout_s", std::to_string(def_timeout)},
            {"servo.max_power_W", std::to_string(max_power_W)}};
  }
};

struct MotorConfigAxis2 {
  float max_acceleration = 0.5f;
  float max_velocity = 1.0f;
  float position_min = -10.0f;
  float position_max = 10.0f;
  float max_current_A = 10.0f; // Used to enforce Max Torque
  float kp = 15.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float max_voltage = 26.0f;
  float max_power_W = 450.0f;
  float def_timeout = std::numeric_limits<double>::quiet_NaN();

  // gear reduction

  std::vector<std::pair<std::string, std::string>> get_configs() const {
    return {{"servo.default_accel_limit", std::to_string(max_acceleration)},
            {"servo.default_velocity_limit", std::to_string(max_velocity)},
            {"servo.max_velocity", std::to_string(max_velocity)}, // Set hard limit same as trajectory
            {"servopos.position_min", std::to_string(position_min)},
            {"servopos.position_max", std::to_string(position_max)},
            {"servo.max_current_A", std::to_string(max_current_A)},
            {"servo.pid_position.kp", std::to_string(kp)},
            {"servo.pid_position.ki", std::to_string(ki)},
            {"servo.pid_position.kd", std::to_string(kd)},
            {"servo.max_voltage", std::to_string(max_voltage)},
            {"servo.default_timeout_s", std::to_string(def_timeout)},
            {"servo.max_power_W", std::to_string(max_power_W)}};
  }
};

struct MotorConfigAxis3 {
  // float max_acceleration = std::numeric_limits<double>::quiet_NaN();
  float max_acceleration = 0.5;
  float max_velocity = 0.5;
  // float max_velocity = std::numeric_limits<double>::quiet_NaN();
  // float position_min = -12.0f;
  // float position_max = 10.0f;
  float position_min = -10.0; //std::numeric_limits<double>::quiet_NaN();
  float position_max = 10.0; //std::numeric_limits<double>::quiet_NaN();
  float max_current_A = 4.0f; // Used to enforce Max Torque
  float kp = 15.0f; //1200 with reduction. accel 3
  float ki = 0.0f;
  float kd = 0.0f;
  float max_voltage = 26.0f;
  float max_power_W = 250.0f;
  // gear reduction
  float def_timeout = std::numeric_limits<double>::quiet_NaN();

  std::vector<std::pair<std::string, std::string>> get_configs() const {
    return {
        {"servo.default_accel_limit", std::to_string(max_acceleration)},
        {"servo.default_velocity_limit", std::to_string(max_velocity)},
        {"servo.max_velocity", std::to_string(max_velocity)}, // Set hard limit same as trajectory
        {"servopos.position_min", std::to_string(position_min)},
        {"servopos.position_max", std::to_string(position_max)},
        {"servo.default_timeout_s", std::to_string(def_timeout)},
        {"servo.max_current_A", std::to_string(max_current_A)},
        {"servo.pid_position.kp", std::to_string(kp)},
        {"servo.pid_position.ki", std::to_string(ki)},
        {"servo.pid_position.kd", std::to_string(kd)},
        {"servo.max_voltage", std::to_string(max_voltage)},
        {"servo.max_power_W", std::to_string(max_power_W)}
    };
  }
};

struct MotorConfigAxis4 {
  float max_acceleration = 20.0f;
  float max_velocity = 10.0f;
  float position_min = -10.0f;
  float position_max = 10.0f;
  float max_current_A = 1.0f; // Used to enforce Max Torque
  float kp = 50.0f;
  float ki = 0.0f;
  float kd = 1.0f;
  float max_voltage = 26.0f;
  float max_power_W = 150.0f;
  // gear reduction

  std::vector<std::pair<std::string, std::string>> get_configs() const {
    return {{"servo.default_accel_limit", std::to_string(max_acceleration)},
            {"servo.default_velocity_limit", std::to_string(max_velocity)},
            {"servo.max_velocity", std::to_string(max_velocity)}, // Set hard limit same as trajectory
            {"servopos.position_min", std::to_string(position_min)},
            {"servopos.position_max", std::to_string(position_max)},
            {"servo.max_current_A", std::to_string(max_current_A)},
            {"servo.pid_position.kp", std::to_string(kp)},
            {"servo.pid_position.ki", std::to_string(ki)},
            {"servo.pid_position.kd", std::to_string(kd)},
            {"servo.max_voltage", std::to_string(max_voltage)},
            {"servo.max_power_W", std::to_string(max_power_W)}};
  }
};

struct MotorConfigAxis5 {
  float max_acceleration = 20.0f;
  float max_velocity = 10.0f;
  float position_min = -10.0f;
  float position_max = 10.0f;
  float max_current_A = 0.5f; // Used to enforce Max Torque
  float kp = 50.0f;
  float ki = 0.0f;
  float kd = 1.0f;
  float max_voltage = 26.0f;
  float max_power_W = 150.0f;
  // gear reduction

  std::vector<std::pair<std::string, std::string>> get_configs() const {
    return {{"servo.default_accel_limit", std::to_string(max_acceleration)},
            {"servo.default_velocity_limit", std::to_string(max_velocity)},
            {"servo.max_velocity", std::to_string(max_velocity)}, // Set hard limit same as trajectory
            {"servopos.position_min", std::to_string(position_min)},
            {"servopos.position_max", std::to_string(position_max)},
            {"servo.max_current_A", std::to_string(max_current_A)},
            {"servo.pid_position.kp", std::to_string(kp)},
            {"servo.pid_position.ki", std::to_string(ki)},
            {"servo.pid_position.kd", std::to_string(kd)},
            {"servo.max_voltage", std::to_string(max_voltage)},
            {"servo.max_power_W", std::to_string(max_power_W)}};
  }
};

struct MotorConfigAxis6 {
  float max_acceleration = 0.20f;
  float max_velocity = 1.5f;
  // float position_min = -10.0f;
  // float position_max = 10.0f;
  float position_min = std::numeric_limits<double>::quiet_NaN();
  float position_max = std::numeric_limits<double>::quiet_NaN();
  float max_current_A = 0.5f; // Used to enforce Max Torque
  float kp = 0.01f;
  float ki = 0.0f;
  float kd = 0.0f;
  float max_voltage = 26.0f;
  float max_power_W = 150.0f;
  float gear_red = (1.0f/66.0f);
  
  // gear reduction

  float def_timeout = std::numeric_limits<double>::quiet_NaN();

  std::vector<std::pair<std::string, std::string>> get_configs() const {
    return {{"servo.default_accel_limit", std::to_string(max_acceleration)},
            {"servo.default_velocity_limit", std::to_string(max_velocity)},
            {"servo.max_velocity", std::to_string(max_velocity)}, // Set hard limit same as trajectory
            {"servopos.position_min", std::to_string(position_min)},
            {"servopos.position_max", std::to_string(position_max)},
            {"servo.max_current_A", std::to_string(max_current_A)},
            {"servo.pid_position.kp", std::to_string(kp)},
            {"servo.pid_position.ki", std::to_string(ki)},
            {"servo.pid_position.kd", std::to_string(kd)},
            {"servo.max_voltage", std::to_string(max_voltage)},
            {"servo.default_timeout_s", std::to_string(def_timeout)},
            // {"motor_position.rotor_to_output_ratio", std::to_string(gear_red)},
            {"servo.max_power_W", std::to_string(max_power_W)}};
  }
};

// TODO implement arm abort

//? Limit switch feedback looks like:
//? sprintf(tmpmsg, "Limit Switch %d, is %d.  \n\r\0", i + 1, get_gpio(axes[i].LIMIT_PIN[0], axes[i].LIMIT_PIN[1]));
