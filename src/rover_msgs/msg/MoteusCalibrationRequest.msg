# Request calibration of one or all moteus motors.
#
# motor_id: 1-based CAN ID (1..6) for a single motor.
#           0 = calibrate all motors in sequence (1 → 2 → ... → 6).
#
# Calibration sequence per motor:
#   1. python3 -m moteus.moteus_tool -t <id> --calibrate
#         --cal-motor-poles 16 --cal-force-kv 265 --cal-hal
#   2. conf set motor_position.sources.0.type type.hall:4
#   3. conf save

int32 motor_id
