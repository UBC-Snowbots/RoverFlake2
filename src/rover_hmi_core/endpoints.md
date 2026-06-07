# ROS Endpoints & HMI Integration Guide

This document catalogs every ROS topic the HMI consumes or produces, the node on the other side of each topic, and a step-by-step recipe for wiring up the **Science HMI modules** (and any other panel that's not yet hooked to real hardware).

> **Last verified:** 2026-05-21. Cross-check against `grep -rE "create_publisher|create_subscription" src/` if the rover stack has changed since.

---

## 1. How the HMI plugs into ROS

All HMI panels live in `rover_hmi_core` as `pluginlib`-loaded `GuiModule` subclasses. There is exactly one ROS node — `rover_hmi` (executable in `rover_hmi_core/src/hmi_host.cpp`) — and **every** panel shares it. Each panel calls `node->create_publisher(...)` / `node->create_subscription(...)` inside its `setNode()` override, so adding a topic to a panel doesn't add a new node.

The host spins ROS callbacks from a 20 ms Qt `QTimer` (~50 Hz). That means:
- A driver publishing at any rate up to ~50 Hz delivers cleanly.
- The HMI is purely a **renderer**; it never owns sensor hardware. To put real data on a panel, an external node has to publish the matching topic.

The legacy GTK package `rover_hmi/` (`main_hmi_node`, `dashboard_hmi_node`, `science_module_hmi_node`) is unrelated to this catalog — it's a separate Glade/gtkmm UI on its own topic set and is out of scope for this document.

---

## 2. Endpoint catalog — every panel, every topic

Topic names are **literal** (the `/` is part of the string). Message types are `<package>/<Type>`.

### Arm

| Panel (file) | Direction | Topic | Type |
|---|---|---|---|
| **MotorStatusModule** (`arm/motor_status_module.cpp`) | sub | `/arm/moteus_feedback` | `rover_msgs/MoteusArmStatus` |
| **PlottingModule** (`arm/plotting_module.cpp`) | sub | `/arm/moteus_feedback` | `rover_msgs/MoteusArmStatus` |
| **MotorConfigModule** (`arm/motor_config_module.cpp`) | sub | `/arm/moteus_feedback` | `rover_msgs/MoteusArmStatus` |
|  | pub | `/arm/config_update` | `rover_msgs/MoteusConfigUpdate` |
|  | pub | `/arm/calibration_request` | `rover_msgs/MoteusCalibrationRequest` |
|  | sub | `/arm/calibration_status` | `rover_msgs/MoteusCalibrationStatus` |
| **SendCommandModule** (`arm/send_command_module.cpp`) | pub | `/arm/command` | `rover_msgs/ArmCommand` |
|  | pub | `/arm/hmi_log` | `std_msgs/String` |
| **CommandLogModule** (`arm/command_log_module.cpp`) | sub | `/arm/hmi_log` | `std_msgs/String` |
|  | sub | `/arm/config_log` | `std_msgs/String` |

**Driver side:** `arm_hardware_interface/moteus_driver_node` (executable: `moteus_driver`) handles all of these. Launch with `ros2 launch arm_hardware_interface arm_hardware_bringup.launch.py`. Status: **fully wired** — plug in the CAN-FD chain and panels light up.

### Drive

| Panel | Direction | Topic | Type |
|---|---|---|---|
| **DriveModule** (`drive/drive_module.cpp`) | pub | `cmd_vel` | `geometry_msgs/Twist` |
| **DrivetrainStopModule** (`electrical/drivetrain_stop_module.cpp`) | pub | `/drivetrain/remote_stop` | `std_msgs/Bool` |
|  | sub | `/drivetrain/remote_stop_status` | `std_msgs/Bool` (`transient_local`) |
| **WheelTelemetryModule** (`electrical/wheel_telemetry_module.cpp`) | sub | `/drivetrain/wheel_states` | `rover_msgs/WheelStates` |
| **VawTreeModule** (`electrical/vaw_tree_module.cpp`) | sub | `/drivetrain/wheel_states` | `rover_msgs/WheelStates` |

**Driver side chain (post-merge of `more_sophisticated_drive_control`):**
```
DriveModule ─cmd_vel─► wheel_speed_node ─left/right_wheel_speeds─► motor_control_node ─Phidget─► 6× BLDC
                                                                       │
DrivetrainStop ◄─/drivetrain/remote_stop───────────────────────────────┤
                ◄─/drivetrain/remote_stop_status (transient_local)─────│
                                                                       │
WheelTelemetry ◄─/drivetrain/wheel_states──────────────────────────────┘
```
Status: **fully wired** — `ros2 launch drive_control drive_the_rover.launch.py`.

### Electrical

| Panel | Direction | Topic | Type | Driver-side status |
|---|---|---|---|---|
| **PowerSummaryModule** (`electrical/power_summary_module.cpp`) | sub | `/power/status` | `rover_msgs/PowerStatus` | ❌ no publisher |
|  | pub | `/power/module_enable` | `rover_msgs/PowerStatus` | ❌ no subscriber |
| **PowerTimelineModule** (`electrical/power_timeline_module.cpp`) | sub | `/power/status` | `rover_msgs/PowerStatus` | (same) |
| **VawTreeModule** (`electrical/vaw_tree_module.cpp`) | sub | `/power/status` | `rover_msgs/PowerStatus` | (same) |
| **LightingModule** (`electrical/lighting_module.cpp`) | pub | `/lighting/control` | `rover_msgs/LightControl` | ❌ no subscriber |
|  | sub | `/lighting/status` | `rover_msgs/LightStatus` | ❌ no publisher |

### Science

| Panel | Direction | Topic | Type | Driver-side status |
|---|---|---|---|---|
| **ScienceModule** (`science/science_module.cpp`) | pub | `/science/command` | `rover_msgs/ScienceModule` | ❌ no subscriber |
| **SciencePipelineModule** (`science/science_pipeline_module.cpp`) | sub | `/science/sensor_data` | `rover_msgs/ScienceSensorData` | ❌ no publisher (gas sensor partial, see §5) |
| **ScienceAnalysisModule** (`science/science_analysis_module.cpp`) | sub | `/science/sensor_data` | `rover_msgs/ScienceSensorData` | (same) |

### MoveIt

| Panel | Direction | Topic | Type | Driver-side status |
|---|---|---|---|---|
| **RvizModule** (`moveit/rviz_module.cpp`) | (spawns `rviz2` + `robot_state_publisher` as child processes) | `/joint_states` (consumed by RViz), URDF param via `robot_description` | `sensor_msgs/JointState` | `/joint_states` is published by `moteus_driver_node` |

---

## 3. Message schemas (the must-know fields)

Field-for-field accuracy matters; the HMI breaks silently if you publish a malformed message.

```
rover_msgs/MoteusArmStatus     # /arm/moteus_feedback (10 Hz)
  rover_msgs/BldcServoConfig[] config
  rover_msgs/BldcServoStatus[] status

rover_msgs/ArmCommand          # /arm/command
  char       cmd_type           # 'p' position, 'v' velocity, 's' stop, ...
  float64[]  positions
  float64[]  velocities
  float64    end_effector
  int16      cmd_value

rover_msgs/WheelStates         # /drivetrain/wheel_states (10 Hz)
  float32[6] speed_rpm          # order: FL FR ML MR RL RR
  float32[6] torque_nm
  float32[6] temperature_c
  float32[6] power_w
  bool[6]    enabled

rover_msgs/PowerStatus         # /power/status (10 Hz target)
  uint8       num_batteries
  float32[3]  battery_voltage_v
  float32[3]  battery_current_a
  float32[3]  battery_power_w
  bool[3]     battery_connected
  float32     total_voltage_v
  float32     total_current_a
  float32     total_power_w
  float32     arm_power_w
  float32     chassis_power_w
  float32     science_power_w
  bool        arm_enabled
  bool        chassis_enabled
  bool        science_enabled

rover_msgs/LightControl        # /lighting/control
  int32  zone                  # 0=Front 1=Back 2=Left 3=Right -1=All
  bool   enabled
  int32  brightness

rover_msgs/LightStatus         # /lighting/status
  bool[4]  enabled              # F B L R
  int32[4] brightness
  int32[4] color_r
  int32[4] color_g
  int32[4] color_b
  bool[4]  auto_mode

rover_msgs/ScienceModule       # /science/command (HMI → embedded board)
  int16 sequenceselection
  int16 sv1status / sv2status / sv3status / sv4status / svf1status / svf2status   # valves
  int16 p1status              # pump
  int16 osf1status / osf2status   # optical flow sensors
  int16 largestatus / smallstatus # dispensers
  int16 carouseldir / carouselindex
  int16 agpowerstatus
  int16 spectrostatus
  int16 light1status / light2status

rover_msgs/ScienceSensorData   # /science/sensor_data (embedded board → HMI, 10 Hz)
  float32 ultrasonic_distance_in     # drill depth, 0..12 inches
  float32 flow_sensor_1_v            # OSF1 voltage
  float32 flow_sensor_2_v
  float32 npk_nitrogen / npk_phosphorus / npk_potassium   # mg/kg
  float32 fluorometer_value          # arbitrary units
  float32 gas_sensor_value           # ppm
  float32[6] spectro_absorbance      # one per vial 1..6
  bool    spectro_ready
```

---

## 4. Adjacent nodes (not consumed by HMI but live on the same broker)

These exist on the bus today; flagging them here so you don't accidentally double-publish a topic that's already taken.

| Node (file) | Direction | Topic | Notes |
|---|---|---|---|
| `arm_control/joy_arm_control` | pub | `/arm/command`, `twist_publisher`, `/joy/feedback` | Joystick-driven arm; competes with `SendCommandModule` on `/arm/command`. Run one at a time. |
| `drive_control/drive_control_node` | pub | `cmd_vel` | Joystick-driven drive; competes with `DriveModule` on `cmd_vel`. |
| `rover_gnss/nmea_reader*` | pub | `gnss_fix` (`sensor_msgs/NavSatFix`) | GNSS reader. |
| `rover_vision/camera_pub_node` | pub | `cam_0..N`, `/camera/{color,infra1,infra2,depth}/image_raw` | Realsense + USB cams. |
| `rover_vision/imu_node`, `full_cameras_node` | pub | `/camera/imu/data` (`sensor_msgs/Imu`) | Realsense IMU. |
| `pose/pose_publisher`, `imu_gps_fuse`, `fusion` | pub | `/estimated_pose`, `/estimated_path`, `/path`, `/odom/fused`, `/odom/fused_path` | Localization. |
| `science_gsensor/send_data_node` | pub | `gas_sensor_data` (`std_msgs/String`) | Raw serial gas-sensor reader. Partially overlaps with science aggregator (see §5). |
| `ptz_cam/*` | mixed | `ipcamera/image_raw`, `/camera/<name>/image_raw`, `*_zoom`, etc. | PTZ camera controls. |

---

## 5. Integration guide — wiring the **Science HMI** to real hardware

The science HMI panels are exactly like the drive panels were before the recent work: the HMI publishes commands and subscribes to sensor data, but **nothing on the rover currently publishes `/science/sensor_data` or consumes `/science/command`**. There's only one related node today — `science_gsensor/send_data_node.py` — and it publishes raw strings on `gas_sensor_data`, not the rich `rover_msgs/ScienceSensorData` the HMI expects.

You need **one new node** (call it `science_aggregator_node`) that:
1. Reads every science instrument over its native transport.
2. Packs the readings into a single `rover_msgs/ScienceSensorData` and publishes it at ~10 Hz.
3. Subscribes to `/science/command` and forwards setpoints to the embedded board (valves, pump, carousel, etc).

Recommended layout: a new package `src/science_aggregator/` with a single Python node. Python is fine here — the data rates are tiny and Python's `pyserial`/`smbus2` make the transport code short.

### 5.1 The instruments

What needs to come through, in `ScienceSensorData` field order:

| Field | Sensor (probable) | Typical transport |
|---|---|---|
| `ultrasonic_distance_in` | HC-SR04 / similar drill-depth sonar | UART/I²C from an Arduino on the science PCB |
| `flow_sensor_1_v`, `flow_sensor_2_v` | OSF1/OSF2 optical flow sensors | analog → ADC on the PCB → serial line |
| `npk_nitrogen / phosphorus / potassium` | NPK soil probe (e.g. JXBS-3001-NPK) | RS-485 Modbus, typically through a USB-RS485 adapter (`/dev/ttyUSB*`) |
| `fluorometer_value` | custom fluorometer board | UART/USB-CDC |
| `gas_sensor_value` | MQ-series or equivalent | UART → already partially handled by `science_gsensor/send_data_node.py` (returns a `String`) |
| `spectro_absorbance[6]` + `spectro_ready` | spectrophotometer + carousel | UART/USB |

For initial bring-up, pick a subset (start with whatever's already on a Pi USB port) and leave the rest at `NaN` — the HMI handles NaN by showing `--`. Field-by-field is fine; everything doesn't need to come up at once.

### 5.2 Skeleton — `science_aggregator_node.py`

The shortest viable form. Save as `src/science_aggregator/science_aggregator/aggregator_node.py`. Replace each `read_*` helper as you bring each sensor online.

```python
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rover_msgs.msg import ScienceSensorData, ScienceModule
# import serial, smbus2, etc. as you bring sensors online

class ScienceAggregatorNode(Node):
    def __init__(self):
        super().__init__('science_aggregator')

        self.sensor_pub_ = self.create_publisher(ScienceSensorData, '/science/sensor_data', 10)
        self.command_sub_ = self.create_subscription(
            ScienceModule, '/science/command', self.on_command, 10)

        # 10 Hz aggregation
        self.timer_ = self.create_timer(0.1, self.publish_sensors)

        # Latest-known values (NaN until a sensor publishes one)
        self._nan = float('nan')
        self._ultrasonic_in = self._nan
        self._osf1_v = self._nan
        self._osf2_v = self._nan
        self._npk = (self._nan, self._nan, self._nan)
        self._fluoro = self._nan
        self._gas = self._nan
        self._spectro = [self._nan]*6
        self._spectro_ready = False

    # ---- read helpers (stubs — replace with real I/O) ----
    def read_ultrasonic(self):    return self._ultrasonic_in
    def read_osf(self):           return self._osf1_v, self._osf2_v
    def read_npk(self):           return self._npk
    def read_fluorometer(self):   return self._fluoro
    def read_gas(self):           return self._gas
    def read_spectro(self):       return self._spectro, self._spectro_ready

    # ---- 10 Hz publish ----
    def publish_sensors(self):
        msg = ScienceSensorData()
        msg.ultrasonic_distance_in = float(self.read_ultrasonic())
        osf1, osf2 = self.read_osf()
        msg.flow_sensor_1_v = float(osf1)
        msg.flow_sensor_2_v = float(osf2)
        n, p, k = self.read_npk()
        msg.npk_nitrogen   = float(n)
        msg.npk_phosphorus = float(p)
        msg.npk_potassium  = float(k)
        msg.fluorometer_value = float(self.read_fluorometer())
        msg.gas_sensor_value  = float(self.read_gas())
        spectro, ready = self.read_spectro()
        msg.spectro_absorbance = [float(x) for x in spectro]
        msg.spectro_ready = bool(ready)
        self.sensor_pub_.publish(msg)

    # ---- command from HMI → embedded board ----
    def on_command(self, msg: ScienceModule):
        # forward valve/pump/carousel/etc. setpoints to whichever embedded
        # board you talk to. For an Arduino over /dev/ttyACM0 a JSON line
        # like {"sv1":1,"p1":0,"cidx":3} is the usual pattern.
        self.get_logger().info(f'cmd seq={msg.sequenceselection} sv1={msg.sv1status}')

def main():
    rclpy.init()
    node = ScienceAggregatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5.3 Package skeleton

```
src/science_aggregator/
├── package.xml
├── setup.py            (ament_python build type)
├── resource/
│   └── science_aggregator
└── science_aggregator/
    ├── __init__.py
    └── aggregator_node.py
```

`package.xml` (essentials):
```xml
<package format="3">
  <name>science_aggregator</name>
  <version>0.0.0</version>
  <description>Reads science instruments, publishes ScienceSensorData, forwards ScienceModule commands</description>
  <maintainer email="rhimaaron@gmail.com">aaron</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <depend>rclpy</depend>
  <depend>rover_msgs</depend>
  <depend>std_msgs</depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

`setup.py` console-script entry:
```python
entry_points={
    'console_scripts': [
        'aggregator_node = science_aggregator.aggregator_node:main',
    ],
},
```
---

## 7. Quick reference — bring everything up

```bash
# Arm
ros2 launch arm_hardware_interface arm_hardware_bringup.launch.py

# Drive
ros2 launch drive_control drive_the_rover.launch.py

# Hmi
ros2 run rover_hmi_core rover_hmi

ros2 run science_aggregator aggregator_node      # once written
ros2 run power_driver power_node                 # once written
ros2 run lighting_driver lighting_node           # once written
```
