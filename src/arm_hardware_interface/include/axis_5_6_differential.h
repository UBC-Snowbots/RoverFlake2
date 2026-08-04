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

// Bench-confirmed convention (2026-08-03): A5 drives both motors the SAME
// direction (m5 = m6 = +v), A6 drives them in OPPOSITION (m5 = +v, m6 = -v).
// The previous version had the two DOFs swapped (an inherited "motors are
// swapped" inversion) — commanding A5 produced A6's motion pattern.
inline void differential_drive(float axis5_input, float axis6_input,
                                float& motor5_output, float& motor6_output)
{
    motor5_output = axis5_input + axis6_input;
    motor6_output = axis5_input - axis6_input;
}


inline void differential_drive_inverse(float motor5_input, float motor6_input,
                                       float& axis5_output, float& axis6_output)
{
    // Exact inverse of differential_drive():
    //   m5 = a5 + a6
    //   m6 = a5 - a6
    // =>
    //   a5 = (m5 + m6) / 2
    //   a6 = (m5 - m6) / 2
    axis5_output = (motor5_input + motor6_input) * 0.5f;
    axis6_output = (motor5_input - motor6_input) * 0.5f;
}