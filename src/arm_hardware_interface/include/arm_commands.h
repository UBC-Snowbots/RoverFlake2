#pragma once

#include <cmath>  // NAN

// =============================================================================
// Arm Command Types  (arm_commands.h)
// =============================================================================
//
// This file defines what commands the ROS layer can send to the driver and
// how those commands are stored internally while they wait to be sent on the
// CAN bus.
//
// Flow:
//   /arm/command topic → commandCallback() → pending_cmds_[]
//                                          → (next poll tick) → active_cmds_[]
//                                          → CAN frame via moteus_protocol.h
//
// Why two arrays (pending / active)?
//   The moteus controller has a watchdog: if it stops receiving valid CAN
//   frames it will fault (fault code 32 = timeout).  active_cmds_ is
//   re-sent every poll cycle so the watchdog never fires, even if the ROS
//   topic goes quiet.  pending_cmds_ just holds the latest user intent
//   until the next poll tick picks it up.
// =============================================================================


// -----------------------------------------------------------------------------
// Command codes — carried in rover_msgs::msg::ArmCommand::cmd_type
// -----------------------------------------------------------------------------

// Stop all motors immediately.
//   - Motor enters kStopped mode → no torque, no position hold (goes limp).
//   - Always accepted, even when a fault is active.
//   - Clears fault-blocking so motion commands can resume afterwards.
constexpr char CMD_STOP    = 'S';

// Absolute position command.
//   positions[]  = target output-shaft revolutions for each motor (NaN = skip that motor)
//   velocities[] = optional velocity feed-forward in rev/s    (NaN = use motion profile)
//   Blocked if the target motor has an active fault — send CMD_STOP first.
constexpr char CMD_ABS_POS = 'P';

// Velocity command (jog mode — hold the button to move).
//   velocities[] = target output-shaft rev/s for each motor (NaN = skip that motor)
//   A velocity of exactly 0.0 is treated as CMD_STOP for that motor
//   (motor goes limp, not hold-position), matching the jog-button release behaviour.
constexpr char CMD_ABS_VEL = 'V';

// Zero (re-home) command — equivalent to "d exact 0" in tview.
//   Resets the position counter to 0.0 at the current physical position.
//   The motor does NOT move — only the position reference is updated.
//   Use this to establish a new zero point after manually positioning the arm.
//   positions[] used as a flag: any non-NaN entry means "zero that motor".
constexpr char CMD_ZERO    = 'Z';


// -----------------------------------------------------------------------------
// Internal command state — one slot per motor
// -----------------------------------------------------------------------------

// Represents one pending or active motion instruction for a single motor.
// Stored in MoteusDriverNode::pending_cmds_[] and active_cmds_[].
struct MotorCommand {
    bool   active     = false;  // true → there is something to send this cycle
    bool   is_stop    = false;  // true → send MakeStop; overrides everything
    bool   is_zero    = false;  // true → send "d exact 0" (re-home in place); one-shot
    double position   = 0.0;   // output-shaft revolutions  (NaN = no position target)
    double velocity   = 0.0;   // output-shaft rev/s        (NaN = use motion profile)
    double max_torque = NAN;   // N·m output-shaft cap      (NaN = firmware default)
};
