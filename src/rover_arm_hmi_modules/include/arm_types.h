// arm_types.h
// Shared constants and plain C++ structs used by arm HMI modules. These are
// decoupled from ROS message types so modules can work with clean data.
//
// NOTE: NUM_MOTORS and the CMD_* constants must stay in sync with
// arm_hardware_interface, which uses the same values on the driver side.

#pragma once

#include <array>
#include <cmath>

constexpr int NUM_MOTORS = 6;

// Command type codes (matching arm driver convention)
constexpr char CMD_ABS_POS = 'P';
constexpr char CMD_ABS_VEL = 'V';
constexpr char CMD_STOP    = 'S';
constexpr char CMD_ZERO    = 'Z';

struct MotorState {
    int id = 0;
    int mode = 0;
    int fault = 0;
    double position = 0.0;
    double velocity = 0.0;
    double torque = 0.0;
    double voltage = 0.0;
    double temperature = 0.0;
    double timestamp = 0.0;
};

struct MotorConfigInfo {
    float kp = 0, ki = 0, kd = 0;
    float max_current = 0;
    float max_velocity = 0;
    float max_acceleration = 0;
    float position_min = 0, position_max = 0;
    float max_voltage = 0, max_power = 0;
    float gear_reduction = 0;
};
