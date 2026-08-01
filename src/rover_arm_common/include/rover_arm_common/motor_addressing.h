#pragma once

#include <cmath>  // M_PI, cstdint

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

enum MotorIndex : uint8_t {
    MOTOR_1_INDEX = 0, // Axis 1 Motor
    MOTOR_2_INDEX = 1, // Axis 2 Motor
    MOTOR_3_INDEX = 2, // Axis 3 Motor
    MOTOR_4_INDEX = 3, // Axis 4 Motor
    MOTOR_5_INDEX = 4, // Axis 5/6 Motor A
    MOTOR_6_INDEX = 5, // Axis 5/6 Motor B
    MOTOR_EE_INDEX = 6, // EE Axis Motor
};

enum AxisIndex : uint8_t {
    AXIS_1_INDEX = 0, // Base
    AXIS_2_INDEX = 1, // Shoulder
    AXIS_3_INDEX = 2, // Elbow
    AXIS_4_INDEX = 3, // Elbow Twist
    AXIS_5_INDEX = 4, // Wrist Pitch
    AXIS_6_INDEX = 5, // Wrist Roll
    AXIS_EE_INDEX = 6, // End Effector Linear Axis
};

enum IkIndex : uint8_t {
    IK_LIN_X_INDEX = 0,
    IK_LIN_Y_INDEX = 1,
    IK_LIN_Z_INDEX = 2,
    IK_ANG_X_INDEX = 3,
    IK_ANG_Y_INDEX = 4,
    IK_ANG_Z_INDEX = 5,
};

constexpr int NUM_AXES = 7;
constexpr int NUM_MOTORS = 7;

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

// urdf joint names only needed for IK/rviz. The URDF still describes the old
// 6-joint arm (a4_rotation..a6_rotation) — a5_differential/a6_differential/
// ee_prismatic don't exist there yet.
static const JointMap ARM_JOINTS[NUM_MOTORS] = {
    //  id   hardware label    urdf joint name    boot angle (rad)   direction
    {  1,   "A1",   "shoulder_joint",       -1.57,          -1.0  },
    {  2,   "A2",   "link_1_joint",         -1.57,          -1.0  },
    {  3,   "A3",   "link1_link2",           0.9,           -1.0  },
    {  4,   "A4",   "a4_rotation",           0.9,           -1.0  },
    {  5,   "A5",   "a5_differential",       0.0,           -1.0  },
    {  6,   "A6",   "a6_differential",       1.2,           -1.0  },
    {  7,   "EE",   "ee_prismatic",          0.0,           -1.0  },
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

inline double revolutionToDegrees(double rev)
{
    return (360.00f / rev);
}

inline double degreesToRevolution(double degrees)
{
    return (degrees / 360.00f);
}