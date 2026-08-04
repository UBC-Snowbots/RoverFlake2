#pragma once

// =============================================================================
// ArmSerialProtocol.h — axis index constants and legacy command codes
// =============================================================================
//
// This header is included by arm_control (controller_config.h) and rover_hmi
// for the AXIS_*_INDEX, IK_*_INDEX, and command code constants.
//
// The serial arm hardware itself has been removed — this file is kept so that
// arm_control and any other packages that #include it continue to compile
// without changes.  Do not add new code here; use arm_commands.h instead.
// =============================================================================

#include <cmath>
#include <limits>
#include <string>
#include <vector>

// Axis array indices — these are the positions in the positions[]/velocities[]
// arrays in rover_msgs/ArmCommand, NOT motor CAN IDs.
static inline constexpr int AXIS_1_INDEX = 0;   // Base
static inline constexpr int AXIS_2_INDEX = 1;   // Shoulder
static inline constexpr int AXIS_3_INDEX = 2;   // Elbow
static inline constexpr int AXIS_4_INDEX = 3;   // Wrist Pitch
static inline constexpr int AXIS_5_INDEX = 4;   // Wrist Roll
static inline constexpr int AXIS_6_INDEX = 5;   // End Effector
static inline constexpr int EE_INDEX     = 6;   // Gripper (extra slot)

// IK axis indices (for delta_twist_cmds / Cartesian servo)
static inline constexpr int IK_LIN_X_INDEX = 0;
static inline constexpr int IK_LIN_Y_INDEX = 1;
static inline constexpr int IK_LIN_Z_INDEX = 2;
static inline constexpr int IK_ANG_X_INDEX = 3;
static inline constexpr int IK_ANG_Y_INDEX = 4;
static inline constexpr int IK_ANG_Z_INDEX = 5;

// Command codes — kept for compatibility with code that uses the old names.
// New code should use the constants in arm_commands.h.
#define HOME_CMD        'h'
#define ABS_POS_CMD     'P'
#define ABS_VEL_CMD     'V'
#define COMM_CMD        'C'
#define TEST_LIMITS_CMD 't'
#define HOME_ALL_ID     50
