#pragma once

#include <cmath>    // NAN
#include <cstdint>  // int16_t

// -----------------------------------------------------------------------------
// Internal command state — one slot per motor
// -----------------------------------------------------------------------------

// Represents one pending or active motion instruction for a single motor.
// Stored in MoteusDriverNode::pending_cmds_[] and active_cmds_[].
struct MotorCommand {
    bool   active     = false;  // true → there is something to send this cycle
    bool   is_stop    = false;  // true → send MakeStop; overrides everything
    bool   motor_space = false; // true → indices 5/6 bypass the wrist transform
    bool   is_zero    = false;  // true → send "d exact 0" (re-home in place); one-shot
    double position   = 0.0;   // output-shaft revolutions  (NaN = no position target)
    double velocity   = 0.0;   // output-shaft rev/s        (NaN = use motion profile)
    double max_torque = NAN;   // N·m output-shaft cap      (NaN = firmware default)
    double max_velocity = NAN;
    double max_acceleration = NAN;
};
