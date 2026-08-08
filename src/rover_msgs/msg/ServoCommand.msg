# One-shot servo motion. The node drives the servo to the commanded position,
# holds it there for hold_ms, then returns it to its rest position (that
# servo's minimum pulse width). A second command for the same servo before the
# hold expires restarts the hold at the new position.

uint8 SERVO_RP=0     # rack-and-pinion drive servo
uint8 SERVO_CLAW=1   # catch/release latch servo

uint8 MODE_LIMIT_MIN=0   # go to the minimum of the calibrated range
uint8 MODE_LIMIT_MAX=1   # go to the maximum of the calibrated range
uint8 MODE_PWM=2         # go to pwm_us, clamped to the calibrated range

uint8 servo
uint8 mode
float32 pwm_us       # requested pulse width in microseconds; MODE_PWM only
uint16 hold_ms       # hold before returning to rest; 0 uses default_hold_ms
