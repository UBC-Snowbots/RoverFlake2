#pragma once

// =============================================================================
// Differential Wrist Drive  (axis_5_6_differential.h)
// =============================================================================
//
// Axes 5 (Wrist Roll) and 6 (End Effector) share a differential drive
// mechanism.  The user commands joint-space values for each axis, but the
// actual motors are coupled: both must move together to achieve a single DOF,
// or in opposite directions for the other DOF.
//
// This function converts joint-space inputs into motor-space commands that
// are then sent over CAN.  It is applied to velocity commands (and position
// commands when axes 5 & 6 are commanded together).
//
// Call site: moteus_driver_node.cpp poll() step 2b for motors at indices 4 & 5.
// =============================================================================


// #define DEBUG_MSGS

inline void differential_drive(float axis5_input, float axis6_input,
                                float& motor5_output, float& motor6_output)
{

    motor5_output = axis6_input + axis5_input;
    motor6_output = -axis6_input + axis5_input;

    // Motors are swapped, so we invert one
    motor6_output = -motor6_output;
}


inline void differential_drive_inverse(float motor5_input, float motor6_input,
                                       float& axis5_output, float& axis6_output)
{
    // Exact inverse of differential_drive():
    //   m5 = a5 + a6
    //   m6 = a6 - a5
    // =>
    //   a5 = (m5 - m6) / 2
    //   a6 = (m5 + m6) / 2
    axis5_output = (motor5_input - motor6_input) * 0.5f;
    axis6_output = (motor5_input + motor6_input) * 0.5f;
}