# Sensor feedback from the rover science subsystem
# Published at ~10 Hz by the science embedded node

float32 ultrasonic_distance_in   # drill depth feedback [0, 12] inches
float32 flow_sensor_1_v          # OSF1 voltage (>threshold = flowing)
float32 flow_sensor_2_v          # OSF2 voltage
float32 npk_nitrogen             # N value mg/kg
float32 npk_phosphorus           # P value mg/kg
float32 npk_potassium            # K value mg/kg
float32 fluorometer_value        # fluorometer output (arb units)
float32 gas_sensor_value         # gas sensor ppm
float32[6] spectro_absorbance    # absorbance per vial (vials 1-6)
bool    spectro_ready            # true when spectro has a fresh reading
