#!/usr/bin/env bash
# Runs every servo_control_node service call in sequence, pausing between each
# so the motion is visible. Ranges: RP 500-2500 us, CLAW 500-1750 us; values
# outside those are clamped by the node, not rejected.
set -euo pipefail

PAUSE=${PAUSE:-2}

# RP servo (servo: 0) to its minimum pulse width
ros2 service call /servo_control_node/set_servo_limit rover_msgs/srv/SetServoLimit "{servo: 0, limit: 0}"
sleep "$PAUSE"

# RP servo to its maximum pulse width
ros2 service call /servo_control_node/set_servo_limit rover_msgs/srv/SetServoLimit "{servo: 0, limit: 1}"
sleep "$PAUSE"

# RP servo to a mid-travel pulse width
ros2 service call /servo_control_node/set_servo_pwm rover_msgs/srv/SetServoPwm "{servo: 0, pwm_us: 1500}"
sleep "$PAUSE"

# CLAW servo (servo: 1) to its minimum pulse width
ros2 service call /servo_control_node/set_servo_limit rover_msgs/srv/SetServoLimit "{servo: 1, limit: 0}"
sleep "$PAUSE"

# CLAW servo to its maximum pulse width
ros2 service call /servo_control_node/set_servo_limit rover_msgs/srv/SetServoLimit "{servo: 1, limit: 1}"
sleep "$PAUSE"

# CLAW servo to a mid-travel pulse width
ros2 service call /servo_control_node/set_servo_pwm rover_msgs/srv/SetServoPwm "{servo: 1, pwm_us: 1100}"
sleep "$PAUSE"

# Park both servos at minimum
ros2 service call /servo_control_node/set_servo_limit rover_msgs/srv/SetServoLimit "{servo: 0, limit: 0}"
ros2 service call /servo_control_node/set_servo_limit rover_msgs/srv/SetServoLimit "{servo: 1, limit: 0}"
