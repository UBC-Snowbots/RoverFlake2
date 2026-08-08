#!/usr/bin/env bash
# Publishes every servo_control_node command mode in sequence, pausing between
# each so the motion is visible. Each command is one-shot: the servo moves,
# holds for hold_ms (default 1000), then returns to its minimum pulse width.
# Ranges: RP 500-2500 us, CLAW 500-1750 us; values outside those are clamped by
# the node, not rejected.
set -euo pipefail

PAUSE=${PAUSE:-2}
TOPIC=/servo_control_node/servo_command
TYPE=rover_msgs/msg/ServoCommand

# RP servo (servo: 0) to its minimum pulse width
ros2 topic pub --once "$TOPIC" "$TYPE" "{servo: 0, mode: 0}"
sleep "$PAUSE"

# RP servo to its maximum pulse width, held for 3 s
ros2 topic pub --once "$TOPIC" "$TYPE" "{servo: 0, mode: 1, hold_ms: 3000}"
sleep "$PAUSE"

# RP servo to a mid-travel pulse width
ros2 topic pub --once "$TOPIC" "$TYPE" "{servo: 0, mode: 2, pwm_us: 1500}"
sleep "$PAUSE"

# CLAW servo (servo: 1) to its minimum pulse width
ros2 topic pub --once "$TOPIC" "$TYPE" "{servo: 1, mode: 0}"
sleep "$PAUSE"

# CLAW servo to its maximum pulse width, held for 3 s
ros2 topic pub --once "$TOPIC" "$TYPE" "{servo: 1, mode: 1, hold_ms: 3000}"
sleep "$PAUSE"

# CLAW servo to a mid-travel pulse width
ros2 topic pub --once "$TOPIC" "$TYPE" "{servo: 1, mode: 2, pwm_us: 1100}"
