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

// FK (Forward Kinematics)
static inline constexpr int AXIS_1_INDEX = 0;
static inline constexpr int AXIS_2_INDEX = 1;
static inline constexpr int AXIS_3_INDEX = 2;
static inline constexpr int AXIS_4_INDEX = 3;
static inline constexpr int AXIS_5_INDEX = 4;
static inline constexpr int AXIS_6_INDEX = 5;

// IK (Inverse Kinematics)
static inline constexpr int IK_LIN_X_INDEX = 0; // Linear Cartesian X
static inline constexpr int IK_LIN_Y_INDEX = 1; // Linear Cartesian Y
static inline constexpr int IK_LIN_Z_INDEX = 2; // Linear Cartesian Z
static inline constexpr int IK_ANG_X_INDEX = 3; // Roll
static inline constexpr int IK_ANG_Y_INDEX = 4; // Pitch
static inline constexpr int IK_ANG_Z_INDEX = 5; // Yaw

// Both IK and FK:
static inline constexpr int EE_INDEX = 6;

// TODO implement arm abort

//? Limit switch feedback looks like:
//? sprintf(tmpmsg, "Limit Switch %d, is %d.  \n\r\0", i + 1, get_gpio(axes[i].LIMIT_PIN[0], axes[i].LIMIT_PIN[1]));
