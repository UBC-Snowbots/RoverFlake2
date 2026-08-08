#!/usr/bin/env bash
set -euo pipefail

TOPIC=/servo_control_node/servo_command
TYPE=rover_msgs/msg/ServoCommand

# GO TO MAX RP PWM
ros2 topic pub --once "$TOPIC" "$TYPE" "{servo: 0, mode: 1}"
sleep 1000

# RP servo to its maximum pulse width, held for 3 s
ros2 topic pub --once "$TOPIC" "$TYPE" "{servo: 0, mode: 2, pwm_us: 1667}"
sleep 5000

# RP servo to a mid-travel pulse width
ros2 topic pub --once "$TOPIC" "$TYPE" "{servo: 0, mode: 1}"
sleep 1000

# CLAW servo (servo: 1) to its minimum pulse width
ros2 topic pub --once "$TOPIC" "$TYPE" "{servo: 1, mode: 1}"
sleep 3000

# # CLAW servo to its maximum pulse width, held for 3 s
# ros2 topic pub --once "$TOPIC" "$TYPE" "{servo: 1, mode: 1, hold_ms: 3000}"
# sleep "$PAUSE"

# # CLAW servo to a mid-travel pulse width
# ros2 topic pub --once "$TOPIC" "$TYPE" "{servo: 1, mode: 2, pwm_us: 1100}"
