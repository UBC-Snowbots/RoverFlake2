/**
 * controller_config.h
 * 
 * Modular controller configuration for arm teleoperation.
 * To switch controllers, change the ACTIVE_CONTROLLER define below.
 * Button/axis indices can be verified by running: ros2 topic echo /joy
 */
#pragma once

// ============ Controller Selection ============
// Set ONE of these as the active controller:
#define CONTROLLER_PRO_CONTROLLER   1
#define CONTROLLER_CYBORG_STICK     2

#define ACTIVE_CONTROLLER CONTROLLER_PRO_CONTROLLER

// ============================================================
//  Nintendo Switch Pro Controller
// ============================================================

//  Button names (used):
//    0 = BTN_EAST        1 = BTN_SOUTH
//    2 = BTN_NORTH       3 = BTN_WEST
//    12 = up
//    13 = down
//    14 = left
//    15 = right
//
//  Axes:
//    0 = Left stick X    1 = Left stick Y
//    2 = Right stick X   3 = Right stick Y
//    4 = ZL analog (rests at 1.0)
//    5 = ZR analog (rests at 1.0)
//
//  NOTE: If your button indices differ, adjust the values below
//        and rebuild. Verify with: ros2 topic echo /joy
// ============================================================

#if ACTIVE_CONTROLLER == CONTROLLER_PRO_CONTROLLER

namespace ControllerConfig {

    // --- Face Buttons ---
    constexpr int BTN_B      = 0;   // East
    constexpr int BTN_A      = 1;   // South
    constexpr int BTN_Y      = 2;   // North
    constexpr int BTN_X      = 3;   // West

    constexpr int BTN_UP     = 12;   // Left shoulder
    constexpr int BTN_DOWN   = 13;   // Right shoulder

    // --- Axes ---
    constexpr int AXIS_LEFT_X  = 0;
    constexpr int AXIS_LEFT_Y  = 1;
    constexpr int AXIS_RIGHT_X = 2;
    constexpr int AXIS_RIGHT_Y = 3;
    // axes[4], axes[5] = ZL/ZR triggers (rest at 1.0, pressed = -1.0) — avoid for motion

    // --- Cartesian Button Mapping (6 buttons → 6 translation directions) ---
    //  Face buttons control X/Y plane, shoulders control Z
    //
    //        Y (+X forward)
    //   X (+Y)         B (-Y)
    //        A (-X backward)
    //
    //   R shoulder → +Z (up)
    //   L shoulder → -Z (down)
    constexpr int BTN_CART_POS_X = BTN_Y;   // North face  → +X (forward)
    constexpr int BTN_CART_NEG_X = BTN_A;   // South face  → -X (backward)
    constexpr int BTN_CART_POS_Y = BTN_X;   // West face   → +Y (left)
    constexpr int BTN_CART_NEG_Y = BTN_B;   // East face   → -Y (right)
    constexpr int BTN_CART_POS_Z = BTN_UP;   // R shoulder  → +Z (up)
    constexpr int BTN_CART_NEG_Z = BTN_DOWN;   // L shoulder  → -Z (down)

    // --- Twist Speed ---
    // command_in_type is "unitless" (-1 to 1), scaled by servo's linear/rotational params
    // 1.0 → full speed (0.5 m/s per servo config)
    constexpr double CART_BUTTON_SPEED = 0.5;  // unitless, range [0.0, 1.0]
    constexpr double ROT_STICK_SPEED   = 0.6;  // unitless, range [0.0, 1.0] for angular

    // --- Deadzone for analog axes (used later for joystick support) ---
    constexpr double AXIS_DEADZONE = 0.15;

    // --- Frame for Cartesian twist commands ---
    // "base_link" = world-fixed directions (forward is always forward)
    // "ee_base_link" = relative to end-effector orientation
    constexpr const char* CART_FRAME_ID = "base_link";

    // --- EE Orientation Stick Mapping ---
    // Left stick  → pitch (tilt forward/back) and yaw (rotate left/right)
    // Right stick → roll  (tilt sideways)
    //
    // Servo angular.x = roll, angular.y = pitch, angular.z = yaw
    constexpr int AXIS_ROLL  = AXIS_RIGHT_X;  // right stick X → roll
    constexpr int AXIS_PITCH = AXIS_LEFT_Y;    // left stick Y  → pitch
    constexpr int AXIS_YAW   = AXIS_LEFT_X;    // left stick X  → yaw
    constexpr bool INVERT_ROLL  = false;
    constexpr bool INVERT_PITCH = false;
    constexpr bool INVERT_YAW   = true;   // typical: push left = positive yaw

    // --- Gripper ---
    // Pro controller trigger is exposed as an analog axis.  Use axis[5]
    // (1.0 at rest, -1.0 when fully pressed) for edge-triggered toggle.
    constexpr int BTN_GRIPPER_TOGGLE = -1;  // button disabled for Pro trigger control
    constexpr int AXIS_GRIPPER_TOGGLE = 5;  // "axis 6" in 1-based indexing
    constexpr double AXIS_GRIPPER_PRESSED_THRESHOLD = 0.0;
    constexpr double GRIPPER_OPEN_VALUE  = 1.0;
    constexpr double GRIPPER_CLOSE_VALUE = 0.0;
    // RViz/FakeSystem gripper command positions (position controller)
    constexpr double GRIPPER_SIM_LEFT_OPEN_POS = 0.035;
    constexpr double GRIPPER_SIM_RIGHT_OPEN_POS = -0.035;
    constexpr double GRIPPER_SIM_LEFT_CLOSE_POS = 0.0;
    constexpr double GRIPPER_SIM_RIGHT_CLOSE_POS = 0.0;
}

#elif ACTIVE_CONTROLLER == CONTROLLER_CYBORG_STICK

// ============================================================
//  Saitek Cyborg USB Stick
// ============================================================
//  Axes:
//    0 = Stick X (left/right)
//    1 = Stick Y (forward/back)
//    2 = Throttle slider (rests non-zero — DO NOT USE for motion)
//    3 = Stick twist/rudder (rotation)
//    4 = Hat X
//    5 = Hat Y
//
//  Buttons:
//    0 = Trigger
// ============================================================

namespace ControllerConfig {

    // --- Axes ---
    constexpr int AXIS_STICK_X   = 0;
    constexpr int AXIS_STICK_Y   = 1;
    constexpr int AXIS_THROTTLE  = 2;  // rests non-zero — skip
    constexpr int AXIS_TWIST     = 3;  // rudder/twist
    constexpr int AXIS_HAT_X     = 4;
    constexpr int AXIS_HAT_Y     = 5;

    // --- Buttons ---
    constexpr int BTN_TRIGGER    = 0;

    // --- Cartesian Button Mapping ---
    // Cyborg stick has no natural button-based Cartesian; set to -1 (disabled)
    constexpr int BTN_CART_POS_X = -1;
    constexpr int BTN_CART_NEG_X = -1;
    constexpr int BTN_CART_POS_Y = -1;
    constexpr int BTN_CART_NEG_Y = -1;
    constexpr int BTN_CART_POS_Z = -1;
    constexpr int BTN_CART_NEG_Z = -1;

    constexpr double CART_BUTTON_SPEED = 0.3;
    constexpr double AXIS_DEADZONE = 0.15;
    constexpr const char* CART_FRAME_ID = "base_link";

    // --- Gripper ---
    constexpr int BTN_GRIPPER_TOGGLE = BTN_TRIGGER;  // trigger button = gripper toggle
    constexpr int AXIS_GRIPPER_TOGGLE = -1;          // no analog trigger axis
    constexpr double AXIS_GRIPPER_PRESSED_THRESHOLD = 0.0;
    constexpr double GRIPPER_OPEN_VALUE  = 1.0;
    constexpr double GRIPPER_CLOSE_VALUE = 0.0;
    constexpr double GRIPPER_SIM_LEFT_OPEN_POS = 0.035;
    constexpr double GRIPPER_SIM_RIGHT_OPEN_POS = -0.035;
    constexpr double GRIPPER_SIM_LEFT_CLOSE_POS = 0.0;
    constexpr double GRIPPER_SIM_RIGHT_CLOSE_POS = 0.0;
}

#else
#error "No valid ACTIVE_CONTROLLER defined in controller_config.h"
#endif
