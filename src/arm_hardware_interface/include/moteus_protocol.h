#pragma once

#include "moteus.h"
#include <cmath>  // NAN, std::isnan

namespace mot = mjbots::moteus;

// =============================================================================
// moteus CAN-FD Protocol  (moteus_protocol.h)
// =============================================================================
//
// This file documents what is actually on the CAN-FD bus and wraps the
// low-level mjbots moteus library calls behind self-documenting functions.
//
// ── CAN-FD ARBITRATION ID (29-bit extended) ──────────────────────────────────
//
//   bits [15:8]  destination motor ID (1–6, matches firmware CAN ID)
//   bits  [7:0]  source host ID       (default 0x7F for the Raspberry Pi host)
//   bits [28:16] flags managed by the library (reply-requested, type bits)
//
//   Example: sending to motor 3 → arbitration ID ≈ 0x0307F
//   Each motor ignores frames not addressed to its own ID.
//
// ── DATA FIELD CONTENTS ──────────────────────────────────────────────────────
//
//   The moteus protocol encodes a sequence of register operations in the
//   data field.  Each operation is a compact (type, register_id, value) tuple.
//   The library builds and parses these — we never touch the raw bytes.
//
//   Frame type          Data field contains
//   ─────────────────── ────────────────────────────────────────────────────
//   QUERY frame         READ  kMode, kPosition, kVelocity, kTorque,
//   (no motion)                kVoltage, kTemperature, kFault
//
//   POSITION frame      WRITE kCommandPosition  (float, NaN → hold current)
//   (motion + query)          kCommandVelocity  (float, output rev/s)
//                             kCommandFeedforwardTorque (float, N·m)
//                             kCommandKpScale   (float, 1.0 = use configured kp)
//                             kCommandKdScale   (float, 1.0 = use configured kd)
//                             kCommandMaxTorque (float, N·m, NaN → firmware default)
//                       READ  kMode, kPosition, kVelocity, kTorque,
//                             kVoltage, kTemperature, kFault
//
//   STOP frame          WRITE kStop (special register — no value; latches kStopped)
//   (disable motor)
//
// ── UNITS ON THE WIRE ────────────────────────────────────────────────────────
//
//   All position/velocity values are in OUTPUT-SHAFT units.
//   The moteus firmware applies the gear reduction internally — you never
//   deal with rotor-side values.
//
//   Position    → output-shaft revolutions (counter resets to 0 on boot)
//   Velocity    → output-shaft rev/s
//   Torque      → N·m at the output shaft
//   Voltage     → bus supply volts
//   Temperature → driver board °C  (NOT motor winding temperature)
//
// ── WATCHDOG / COMMAND PERSISTENCE ──────────────────────────────────────────
//
//   The moteus has a configurable watchdog timeout (servo.default_timeout_s).
//   If valid CAN frames stop arriving, it faults with code 32 (Timeout).
//   A QUERY frame counts as a valid keep-alive — the driver always sends
//   at least one frame per motor per poll cycle for this reason.
// =============================================================================

namespace MoteusProtocol {

// Build a query-only frame for one motor.
// Sends no motion command.  The motor replies with current telemetry.
// Use this every cycle for motors that have no active command — it keeps
// the watchdog happy without accidentally commanding motion.
inline mot::CanFdFrame makeQueryFrame(mot::Controller& ctrl) {
    return ctrl.MakeQuery();
}

// Build a position/velocity command frame for one motor.
//
//   position   — target output-shaft revolutions.
//                NaN → no position target (pure velocity mode).
//
//   velocity   — target output-shaft rev/s.
//                NaN → let the firmware use its motion profile velocity limit.
//                Combine with a position target to cap approach speed.
//
//   max_torque — output-shaft torque cap in N·m.
//                NaN → use the firmware default (set by motor_config.h).
//
// The motor also sends back a full telemetry reply (same as makeQueryFrame).
// This frame must be re-sent every poll cycle while motion is desired —
// a single frame does NOT latch; the motor will stop if frames stop arriving.
inline mot::CanFdFrame makePositionFrame(mot::Controller& ctrl,
                                            double position,
                                            double velocity,
                                            double max_velocity = NAN,
                                            double max_acceleration = NAN,
                                            double max_torque = NAN) {
    mot::PositionMode::Command cmd;
    cmd.position = position;
    cmd.velocity = velocity;
    if (!std::isnan(max_velocity))
        cmd.velocity_limit = max_velocity;    
    if (!std::isnan(max_acceleration))
        cmd.accel_limit = max_acceleration; 
    if (!std::isnan(max_torque))
        cmd.maximum_torque = max_torque;
    return ctrl.MakePosition(cmd);
}

// Build a stop frame for one motor.
// Puts the motor into kStopped mode: driver disabled, zero torque, motor goes
// limp.  To resume motion you must send a position frame afterwards.
// This is one-shot — do NOT repeat it every cycle or the motor stays limp
// permanently.  After sending, the driver reverts to makeQueryFrame each cycle.
inline mot::CanFdFrame makeStopFrame(mot::Controller& ctrl) {
    return ctrl.MakeStop();
}

// Decode a reply frame received from a motor into structured telemetry.
// The returned Query::Result fields map directly to MotorTelem (arm_telemetry.h).
inline mot::Query::Result parseReply(const mot::CanFdFrame& frame) {
    return mot::Query::Parse(frame.data, frame.size);
}

} // namespace MoteusProtocol
