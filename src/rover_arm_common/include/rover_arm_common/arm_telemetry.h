#pragma once

// =============================================================================
// Motor Telemetry  (arm_telemetry.h)
// =============================================================================
//
// Per-motor state decoded from CAN reply frames each poll cycle.
//
// The moteus controller sends a reply frame in response to every command or
// query frame.  The driver calls MoteusProtocol::parseReply() on each reply
// and fills one MotorTelem slot.
//
// UNITS — all values are at the OUTPUT SHAFT:
//   The moteus firmware applies the gear ratio internally.
//   Everything the driver sees is already scaled to output-shaft units.
//   If you want rotor-side values, multiply by the gear_reduction stored
//   in MotorConfig.
//
// CONNECTED FLAG:
//   Set to false at the start of every poll cycle, then set to true only
//   when a reply frame actually arrives for that motor.  If a motor misses
//   a cycle its telemetry fields still hold the last known values, but
//   connected == false so you know they may be stale.
// =============================================================================

struct MotorTelem {
    float position    = 0;  // Output-shaft revolutions since power-on.
                            //   Note: not absolute — resets to 0 on boot,
                            //   no homing sensor.  INITIAL_POSITIONS[] in
                            //   motor_addressing.h provides the zero-offset.

    float velocity    = 0;  // Output-shaft velocity in rev/s.

    float torque      = 0;  // Output-shaft torque in N·m.

    float voltage     = 0;  // CAN bus supply voltage in volts.

    float q_current   = 0;  // Phase (Q-axis) current in amps — the torque-producing
                            //   component of motor current.  Requires the query format
                            //   to request kQCurrent (kFloat resolution).

    float power       = 0;  // Instantaneous electrical power draw in watts.
                            //   Requires the query format to request kPower (kFloat).

    float temperature = 0;  // Driver board temperature in °C.
                            //   (Not motor winding temp — the driver's own FET temp.)

    int   mode        = 0;  // moteus operating mode enum:
                            //   0  = kStopped   — driver disabled, no torque
                            //   1  = kFault     — fault latched, see fault field
                            //   5  = kPosition  — active position/velocity control
                            //   (other values defined in moteus.h Query::Mode)

    int   fault       = 0;  // moteus fault code (0 = no fault):
                            //    1 = DmaStreamTransferError
                            //   32 = Timeout  ← most common: watchdog fired because
                            //                   CAN frames stopped arriving
                            //   33 = CalibrationFault
                            //   (full list in mjbots/moteus documentation)

    bool  limit_switch = false; // Limit switch state (1 = pressed / wire broken)
    bool  connected   = false; // Did we receive a reply this poll cycle?
};
