# Full rover power status snapshot — published at ~10 Hz
uint8    num_batteries
float32[3] battery_voltage_v
float32[3] battery_current_a
float32[3] battery_power_w
bool[3]    battery_connected
float32    total_voltage_v
float32    total_current_a
float32    total_power_w
float32    arm_power_w
float32    chassis_power_w
float32    science_power_w
bool       arm_enabled
bool       chassis_enabled
bool       science_enabled
