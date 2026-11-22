#!/usr/bin/env bash
set -euo pipefail

TOPIC=/servo_control_node/servo_command
TYPE=rover_msgs/msg/ServoCommand
SRV=/servo_control_node

# Open / close the CLAW servo (holds position until the next command)
ros2 service call "$SRV/claw/open" std_srvs/srv/Trigger
sleep 2
ros2 service call "$SRV/claw/close" std_srvs/srv/Trigger
sleep 2

# RP servo to its maximum pulse width (returns to rest after default_hold_ms)
ros2 topic pub --once "$TOPIC" "$TYPE" "{servo: 0, mode: 1}"
sleep 2

# RP servo to a chosen pulse width, held for 3 s
ros2 topic pub --once "$TOPIC" "$TYPE" "{servo: 0, mode: 2, pwm_us: 1667, hold_ms: 3000}"
sleep 4

# CLAW servo to a mid-travel pulse width, held for 2 s
ros2 topic pub --once "$TOPIC" "$TYPE" "{servo: 1, mode: 2, pwm_us: 1200, hold_ms: 2000}"
sleep 3
