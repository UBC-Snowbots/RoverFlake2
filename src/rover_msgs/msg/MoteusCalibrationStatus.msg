# Published by the moteus driver after each calibration step change.
#
# state:
#   0 = idle
#   1 = running
#   2 = success
#   3 = failed

int32 CALIB_IDLE    = 0
int32 CALIB_RUNNING = 1
int32 CALIB_SUCCESS = 2
int32 CALIB_FAILED  = 3

int32  motor_id   # 1-based CAN ID
int32  state
string message
