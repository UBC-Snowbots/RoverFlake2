# Real-time configuration update for a single moteus motor register.
# Published by the HMI Motor Params panel; subscribed by moteus_driver_node.
# The driver calls DiagnosticCommand("conf set <register_name> <value>") and
# also updates its in-memory MotorConfig so subsequent feedback messages
# reflect the change.
#
# motor_id      — 0-based motor index (0 = Base, 1 = Shoulder, ..., 5 = End Effector)
# register_name — moteus firmware register, e.g. "servo.pid_position.kp"
# value         — new value to set (NaN = no change / cancel)

int32 motor_id
string register_name
float32 value
