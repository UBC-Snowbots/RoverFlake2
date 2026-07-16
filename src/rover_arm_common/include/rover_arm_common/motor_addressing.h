#pragma once

#include <cmath>  // M_PI

// =============================================================================
// Motor Addressing & Joint Mapping  (motor_addressing.h)
// =============================================================================
//
// This is the single source of truth for how CAN motor IDs map to physical
// joints, URDF joints, and the direction convention between them.
//
// MOTOR IDs
//   Each moteus controller is flashed with a unique CAN ID (1–6).
//   The ID is set via the moteus tool and stored in the firmware.
//   Array index = motor_id - 1.
//
// DIRECTION SIGNS
//   The moteus reports positive position when the rotor moves in one direction.
//   Depending on how the motor is mounted, that might be the opposite of what
//   the URDF considers positive.  direction = +1 or -1 corrects that.
//
// INITIAL POSITIONS
//   The moteus resets its position counter to 0 on every boot (no absolute
//   encoder — it uses a relative Hall sensor).  initial_pos_rad is the known
//   joint angle when the arm is in the power-on pose, so we can still publish
//   a physically meaningful /joint_states for RViz.
// =============================================================================

enum class MotorIndex : int {
    MOTOR_1 = 0, // Axis 1 Motor
    MOTOR_2 = 1, // Axis 2 Motor
    MOTOR_3 = 2, // Axis 3 Motor
    MOTOR_4 = 3, // Axis 4 Motor
    MOTOR_5 = 4, // Axis 5/6 Motor A
    MOTOR_6 = 5, // Axis 5/6 Motor B
    MOTOR_EE = 6, // EE Axis Motor
};

enum class AxisIndex : int {
    AXIS_1 = 0, // Base
    AXIS_2 = 1, // Shoulder
    AXIS_3 = 2, // Elbow
    AXIS_4 = 3, // Elbow Twist
    AXIS_5 = 4, // Wrist Pitch
    AXIS_6 = 5, // Wrist Roll
    AXIS_EE = 6, // End Effector Linear Axis
};
constexpr int NUM_AXES = 7;


constexpr int NUM_MOTORS = 6;

// Gripper finger joints have no motors.  They are published as static 0 in
// /joint_states so robot_state_publisher doesn't warn about missing joints.
static const char* GRIPPER_JOINT_NAMES[] = {
    "finger_left_joint",
    "finger_right_joint",
};
constexpr int NUM_GRIPPER_JOINTS = 2;

// One entry per motor.  Array index = motor_id - 1.
struct JointMap {
    int         motor_id;         // CAN bus address programmed into the moteus firmware
    const char* hardware_name;    // Human-readable label — used in log messages
    const char* urdf_joint_name;  // Must match the <joint name="..."> in dev_arm.urdf
    double      initial_pos_rad;  // Joint angle (rad) when position counter reads 0 at boot
    double      direction;        // +1 or -1: sign between output revolutions and URDF angle
};

//!? this is wrong?? where is A4?
static const JointMap ARM_JOINTS[NUM_MOTORS] = {
    //  id   hardware label    urdf joint name    boot angle (rad)   direction
    {  1,   "Base",           "shoulder_joint",       -1.57,          -1.0  },
    {  2,   "Shoulder",       "link_1_joint",         -1.57,          -1.0  },
    {  3,   "Elbow",          "link1_link2",           0.9,           -1.0  },
    {  4,   "Wrist Pitch",    "a4_rotation",           0.0,           -1.0  },
    {  5,   "Wrist Roll",     "a5_rotation",           1.2,           -1.0  },
    {  6,   "End Effector",   "a6_rotation",           0.0,           -1.0  },
};


// -----------------------------------------------------------------------------
// Unit conversion helpers
// -----------------------------------------------------------------------------

// Convert motor telemetry (output revolutions) to URDF joint angle (radians).
//
// moteus reports position in OUTPUT-SHAFT revolutions.  The gear ratio is
// applied internally by the firmware, so this function does NOT need to
// account for it.  We just scale by 2π and add the boot-time offset.
inline double motorRevToJointRad(int motor_idx, double output_revolutions) {
    const auto& j = ARM_JOINTS[motor_idx];
    return j.initial_pos_rad + (j.direction * output_revolutions * 2.0 * M_PI);
}

// Convert output-shaft rev/s to URDF joint velocity (rad/s).
inline double motorRevPerSecToJointRadPerSec(int motor_idx, double output_rev_per_sec) {
    return ARM_JOINTS[motor_idx].direction * output_rev_per_sec * 2.0 * M_PI;
}
