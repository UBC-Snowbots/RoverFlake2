# learn_drive: autonomy development package (Tim)

Status as of **2026-11-24**. Branch: `tim/autonav-dev`.

This package holds everything built so far for autonomous driving: a Nav2 setup for our rover that runs in a fake world on a laptop, a driver for the Scanse Sweep lidar, an IMU demo, and a script that drives a square on its own (tested on the real mini rover). It also documents how to bring up the mini rover's drive and what was learned testing it.

None of this is wired into the team's main launch files yet. It is development work meant to be reused.

---

## 1. Overview

### What works today

| Piece | What it does | Runs on | Status |
|---|---|---|---|
| **Nav2 simulation** (`rover_nav_sim.launch.py`) | A pretend rover in a pretend rock field. Click a goal in RViz and Nav2 plans a path, drives there and avoids rocks, with recovery moves if it gets stuck | Any laptop, no hardware | Works |
| **Nav2 with the real lidar** (same launch, `lidar:=real`) | The real Sweep lidar sees the real room; the pretend rover drives around the real walls and objects | Laptop + Sweep lidar | Works |
| **Sweep lidar driver** (`sweep_driver`) | Reads the Scanse Sweep over USB and publishes `/scan` | Laptop + Sweep | Works (the unit itself degrades after a few minutes, see section 7) |
| **Sweep checker** (`scripts/sweep_check.py`) | Talks to the Sweep without ROS and reports whether it is returning good readings | Laptop + Sweep | Works |
| **IMU demo** (`imu_viz.launch.py`) | A rover model in RViz turns and moves the way you move the Phidget IMU in your hand | Laptop + Phidget IMU | Works (demo only; an IMU can't measure distance for real navigation) |
| **Square drive** (`scripts/square_drive.py`) | Drives a square on its own: straight sides, 90° corners measured with the IMU | Team Unity sim (SimFlake) and the **real mini rover** | Works. Full 1 m square on the mini rover on 2026-11-22 |
| **Lidar follow** (`scripts/lidar_follow.py`) | Follows a person using only the lidar, keeping 2 m away; stops if it loses them | Laptop simulation (`person:=true`) and the mini rover | Tested in simulation; first real test planned for the next session |
| **Mini rover bring-up** (section 4.6) | Step-by-step: CAN drive, IMU, test commands, square | The mini rover's onboard computer | Works |

### What is not done yet (for the Assist Astronaut task, #243)

The agreed split: **Pranav** handles perception (finding the astronaut with the camera), the commands, and gluing the pieces together. **Tim** handles driving the rover to the astronaut's position: stopping about 2 m away, following as the astronaut moves, and avoiding obstacles.

Missing on the driving side:

1. **Agreeing the handoff message** with Pranav (topic name, message type, frame, rate, what happens when the astronaut is lost). A proposal is in section 8.
2. **A follow node**: takes each astronaut position and keeps the Nav2 goal 2 m short of it. Not built yet; can be built and tested in the simulation with a fake astronaut.
3. **Odometry on the real mini rover**: it has no wheel encoders, so nothing reports how far it moved. Nav2 can't run without this.
4. **A Nav2 setup for the real mini rover**: the simulation launch without the pretend rover and pretend lidar, with the mini rover's real size and speeds.
5. **Sensors running on the rover**: the lidar driver on the rover's computer, and measured mount positions for the lidar, IMU and camera.
6. **Motor direction fix** (team hardware/firmware): the mini rover turns the wrong way (section 6).

---

## 2. Setup on a new computer

Needs **Ubuntu 24.04** and **ROS 2 Jazzy**. The repo is assumed to be cloned at `~/RoverFlake2`; change the paths if yours is elsewhere.

### Install the packages this uses (once)

```bash
sudo apt update
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher ros-jazzy-xacro \
  ros-jazzy-leo-description \
  ros-jazzy-phidgets-spatial ros-jazzy-imu-filter-madgwick \
  ros-jazzy-teleop-twist-keyboard \
  python3-serial python3-numpy
```

- `navigation2` / `nav2-bringup`: Nav2 and its RViz panel.
- `leo-description`: the Leo rover model used as a stand-in in the simulation (it has wheels that spin in RViz).
- `phidgets-spatial`, `imu-filter-madgwick`: the IMU driver and the filter that turns raw IMU readings into a heading.
- `python3-serial`: the Sweep lidar talks over a USB serial port.

To use the Sweep lidar without `sudo`, add yourself to the serial-port group once, then log out and back in:

```bash
sudo usermod -aG dialout $USER
```

### Build (once, and again after adding new files)

```bash
source /opt/ros/jazzy/setup.bash
cd ~/RoverFlake2
colcon build --symlink-install --packages-select learn_drive rover_description
```

`rover_description` is only needed for the IMU demo's team rover model.

### In every new terminal

```bash
source /opt/ros/jazzy/setup.bash
source ~/RoverFlake2/install/setup.bash
```

**Working alone on a laptop?** Also run `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. Otherwise your simulation can see other people's nodes on the same WiFi, and theirs can see yours. Don't use this when talking to the real rover over the network.

---

## 3. What's in the package

```
learn_drive/
├── learn_drive/
│   ├── fake_rover.py          pretend rover: obeys /cmd_vel, publishes /odom + TF
│   ├── fake_lidar.py          pretend lidar in a pretend rock field (optionally with a walking person): publishes /scan
│   ├── sweep_driver.py        Scanse Sweep lidar driver: publishes /scan
│   ├── imu_rover_motion.py    IMU demo: rover model turns and moves with the IMU
│   └── imu_to_rover_tf.py     IMU demo, tilt/turn only
├── launch/
│   ├── rover_nav_sim.launch.py   Nav2 + pretend rover + pretend or real lidar + RViz
│   └── imu_viz.launch.py         IMU driver + filter + rover model + RViz
├── config/
│   ├── rover_nav2.yaml        all Nav2 settings
│   ├── rover_nav.rviz         RViz layout for the Nav2 simulation
│   ├── lidar_view.rviz        RViz layout for the lidar "radar" view
│   └── imu.rviz               RViz layout for the IMU demo
└── scripts/                   run directly with python3 (not built)
    ├── square_drive.py        drives a square on its own
    ├── lidar_follow.py        follows a person using only the lidar
    └── sweep_check.py         Sweep lidar check, no ROS needed
```

Programs you start with `ros2 run learn_drive <name>`: `fake_rover`, `fake_lidar`, `sweep_driver`, `imu_rover_motion`, `imu_to_rover_tf`.

---

## 4. How to run each piece

Every block below assumes a fresh terminal that has run the two `source` lines from section 2. A program that keeps running needs its own terminal.

### 4.1 Nav2 simulation (no hardware)

```bash
ros2 launch ~/RoverFlake2/src/learn_drive/launch/rover_nav_sim.launch.py
```

RViz opens, looking down on the rover at the start point.

- **Send a goal:** click **2D Goal Pose** in RViz's top bar, then click and drag on the map. Nav2 plans a path (shown as a line) and drives there.
- **Add a rock:** click **2D Pose Estimate** (reused for this, since there's no map to localize against) or **Publish Point**, then click on the map. The pretend lidar sees it immediately.
- **Person mode:** add `person:=true` to the launch command for an open field with a person in it (blue cylinder), for testing `lidar_follow.py` (section 4.7). In this mode **Publish Point** makes the person walk to the clicked spot instead of adding a rock, and 40% of lidar directions get no reading, like the Sweep.
- **What RViz shows:**
  - Global and local costmaps: the grids Nav2 uses to judge where is safe. Bright cells are obstacles; the fading halo around them is the keep-away margin.
  - Planned Path (`/plan`): the route Nav2 found.
  - Where It Drove (`/rover_trail`): the path the rover actually took.
  - Carrot (`/lookahead_point`): the point on the path the controller is steering toward.
  - Lidar Hits (`/scan`): what the pretend lidar sees.
  - Real Rocks (`/rocks`): where the rocks really are. Nav2 never sees this; it only sees lidar hits.

Send a goal from the command line instead of RViz (positions in metres from the start point, x forward, y left):

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: odom}, pose: {position: {x: 5.0, y: 1.0}, orientation: {w: 1.0}}}}" --feedback
```

### 4.2 Nav2 with the real Sweep lidar (pretend rover, real room)

The real lidar sits still on the floor where the pretend rover starts. The pretend rover drives around whatever the lidar sees in the room.

Terminal 1, the lidar driver (see 4.3 for options):

```bash
ros2 run learn_drive sweep_driver
```

Terminal 2:

```bash
ros2 launch ~/RoverFlake2/src/learn_drive/launch/rover_nav_sim.launch.py lidar:=real
```

- If the room appears rotated the wrong way round, add `lidar_yaw:=180` (the direction the lidar faces, in degrees).
- To see what settings a launch file accepts: `ros2 launch <file> --show-args`. `ros2 launch` silently ignores settings it doesn't know, so check this if a setting seems to do nothing.
- Tuning tips for small rooms: in `config/rover_nav2.yaml`, lower `inflation_radius` to 0.4–0.5 (otherwise the keep-away halos fill the room), and raise the local costmap's `obstacle_max_range` to 6.0.

### 4.3 Sweep lidar on its own

Find which port it's on. Plug it in, then:

```bash
ls /dev/ttyUSB*
```

**Check it works (no ROS):**

```bash
python3 ~/RoverFlake2/src/learn_drive/scripts/sweep_check.py /dev/ttyUSB0
```

It starts the Sweep, reads for 5 seconds, and prints how many readings came back, how many were good, how many were flagged as errors or had no real echo, and a verdict.

**Run the driver** (publishes `/scan`, frame `laser`):

```bash
ros2 run learn_drive sweep_driver
ros2 run learn_drive sweep_driver --ros-args -p port:=/dev/ttyUSB1 -p motor_hz:=2 -p sample_rate:=500
```

| Setting | Default | Meaning |
|---|---|---|
| `port` | `/dev/ttyUSB0` | Serial port |
| `frame_id` | `laser` | Frame name in the published scans |
| `motor_hz` | 5 | Turns per second (1–10). Slower gives more points per turn. Changing it makes the motor re-settle for about 10 s |
| `sample_rate` | 1000 | Distance readings per second: 500, 750 or 1000 |
| `angle_bins` | 360 | Slots per turn in the published scan (1° each) |
| `range_min` / `range_max` | 0.1 / 40.0 | Metres. Readings outside this are thrown out |
| `min_strength` | 10 | Echoes weaker than this are treated as junk |
| `flip` | False | Set True if the room appears mirror-imaged |

Every 5 seconds the driver prints scans per second, points per turn, readings per second (received, good, flagged), and why readings were thrown out ("too close", "nothing came back", "echo too weak"). When it was healthy at 2 turns per second it gave about 320 good readings per second.

**Radar view in RViz:**

```bash
rviz2 -d ~/RoverFlake2/src/learn_drive/config/lidar_view.rviz
```

It's centred on the lidar, with a 1 m grid. The red axis is the lidar's front. Green squares are lidar hits.

### 4.4 IMU demo (Phidget IMU plugged into the laptop)

```bash
mkdir -p ~/.rviz2 && cp ~/RoverFlake2/src/learn_drive/config/imu.rviz ~/.rviz2/imu.rviz   # once
ros2 launch ~/RoverFlake2/src/learn_drive/launch/imu_viz.launch.py
```

It starts the Phidgets driver (`/imu/data_raw`), the Madgwick filter (`/imu/data`, orientation filled in), a rover model and RViz.

| Setting | Default | Meaning |
|---|---|---|
| `model` | `team` | `team` = our rover model (needs `rover_description` built). `leo` = Leo rover, whose wheels spin like a differential drive |
| `motion` | `true` | `true`: pushes on the IMU move the rover. `false`: turning and tilting only |
| `scale` | 3.0 | The rover moves this many times as far as your hand (1.0 = life size) |
| `flat` | `true` | `true`: the rover stays flat on the ground and only turns. `false`: shows tilt too |
| `use_mag` | `false` | Use the compass. Off by default, because nearby metal and magnets bend it |

Example: `ros2 launch .../imu_viz.launch.py model:=leo scale:=1.0`. Reset the rover to the origin with `ros2 service call /reset_position std_srvs/srv/Trigger`.

How the IMU board was held relative to the model is set in the launch file (`MOUNT`: roll 180°, pitch 0°, yaw 90°). That's for the demo only; the IMU on the mini rover needs its own measurement.

### 4.5 Square drive in the team's Unity sim (SimFlake)

With SimFlake playing (it publishes `/cmd_vel`, `/odom`, `/imu/data`, `/gps/fix`, `/scan`, `/camera/...`):

```bash
cd ~/RoverFlake2/src/learn_drive/scripts
python3 square_drive.py
```

Comparison run, with every move timed by the clock and no sensor feedback:

```bash
python3 square_drive.py --ros-args -p distance_source:=time -p heading_source:=time
```

In SimFlake, the GPS line printed at the end is the rover's true position, so "ended X m from the start" is the honest score. Script details are in section 5.5.

### 4.6 The real mini rover

The mini rover's onboard computer is called `nubs` (Ubuntu 24.04, ROS 2 Jazzy). Its address on the team WiFi was `172.30.0.4` on 2026-11-22; ask the team if it has changed.

**Run everything on the rover's computer over ssh, not on your laptop.** Over the WiFi, pings took 200–490 ms, and the rover's motors stop if they don't get a command for 0.5 s.

Open several terminals on your laptop and run `ssh rv@<rover address>` in each.

**Terminal 1: CAN.** Check the USB-CAN adapter is there and set to 500 kbps:

```bash
ip -details link show can0
```

- "Device does not exist": the adapter isn't plugged into the rover's computer.
- Not `UP`, or not `bitrate 500000`: set it up:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

Then start the CAN interface and leave it running:

```bash
ros2 launch arm_hardware_interface generic_can_interface_bringup_on_board.launch.py
```

**Terminal 2: the translator** (turns `/cmd_vel` into CAN drive messages). Leave it running:

```bash
ros2 run arm_hardware_interface mini_rover_translator
```

**Terminals 3 and 4: the IMU.** Driver, then filter:

```bash
ros2 launch phidgets_spatial spatial-launch.py
```

```bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_mag:=false -p publish_tf:=false -p world_frame:=enu
```

Keep the rover still for about 10 s while these start.

**Terminal 5: check everything is up**

```bash
ros2 topic list                         # expect /cmd_vel, /imu/data, /imu/data_raw, /can/main/...
ros2 topic echo /can/main/heartbeats    # the rover's motor board sends one per second
ros2 topic hz /imu/data                 # should be a steady rate
```

If no heartbeats arrive, the computer can't reach the rover's motor board (section 7, CAN).

**Short test commands.** Each one sends 20 commands per second for 3 seconds, then stops; the rover stops on its own 0.5 s later.

```bash
ros2 topic pub -r 20 -t 60 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"      # forward
ros2 topic pub -r 20 -t 60 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.0}}"     # turn (+ = left in ROS)
```

**Heading watcher.** Prints the IMU heading in degrees a few times a second:

```bash
python3 -c "
import math, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
rclpy.init(); n = rclpy.create_node('heading_watch'); k = [0]
def cb(m):
    k[0] += 1
    if k[0] % 20: return
    q = m.orientation
    print('heading %.0f deg' % math.degrees(math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))))
n.create_subscription(Imu, '/imu/data', cb, qos_profile_sensor_data)
rclpy.spin(n)"
```

**Flip relay (needed until the turn direction is fixed, section 6).** It takes commands on `/cmd_vel_square`, flips the turn direction, and sends them on to `/cmd_vel`. Leave it running in its own terminal while the square script runs. Stop it before using teleop, or teleop's turns will be reversed too.

```bash
python3 -c "
import rclpy
from geometry_msgs.msg import Twist
rclpy.init(); n = rclpy.create_node('flip_turn')
pub = n.create_publisher(Twist, '/cmd_vel', 10)
def cb(m):
    m.angular.z = -m.angular.z
    pub.publish(m)
n.create_subscription(Twist, '/cmd_vel_square', cb, 10)
rclpy.spin(n)"
```

**Run the square.** Copy the script to the rover first (from your laptop: `scp ~/RoverFlake2/src/learn_drive/scripts/square_drive.py rv@<rover address>:~/`). Then, on the rover:

```bash
python3 ~/square_drive.py --ros-args -p cmd_vel_topic:=/cmd_vel_square -p distance_source:=time \
  -p side_length:=1.0 -p linear_speed:=0.6 -p min_turn_speed:=1.0 -p turn_speed:=1.3 -p heading_tolerance_deg:=4.0
```

- `distance_source:=time`: the mini rover has no odometry, so each side is driven for side length ÷ speed seconds.
- `linear_speed:=0.6`, `min_turn_speed:=1.0`: the values that worked on 2026-11-22. Lower ones may also work on smooth floors (section 6).
- `heading_tolerance_deg:=4.0`: accept a corner within 4° of 90°. The run on 2026-11-22 used 10°, which stopped every corner at about 81°. Use 6 if it rocks back and forth at a corner.
- Add `-p gps_topic:=/gnss_fix` if you want the end-of-run GPS line. The rover's GPS publishes there, not on `/gps/fix`. GPS error (metres) is bigger than this square, though, so score it with a tape measure instead.
- To get true side lengths: tape one side, then set `side_length` to (wanted length) × (side_length used) ÷ (measured length).


### 4.7 Follow a person with the lidar (`scripts/lidar_follow.py`)

It only uses `/scan` and `/cmd_vel`. No odometry, Nav2, GPS or TF is needed, because it reacts to what the lidar sees right now ("person 3 m away, 10° left").

How it works:

1. **It only looks in front.** It keeps readings within ±60° of straight ahead and further than 0.35 m. The rest is the rover's own body, since the lidar sits at the front middle.
2. **It groups readings into blobs.** Neighbouring readings at similar distances form a blob. Blobs wider than 0.9 m (walls, furniture) are ignored, so the small blobs left are person-sized.
3. **It locks on.** At the start, and again after losing you, it locks onto the closest person-sized blob within ±30° of straight ahead and within 3 m. After that, every lidar turn it picks the person-sized blob nearest to where you were. It also allows for how the rover itself moved since the last turn.
4. **It drives.**
   - It turns toward you at 1.5 × the angle off-centre (in rad/s), capped at 1.0 rad/s. Within 8° of straight ahead it doesn't turn.
   - It drives forward at 0.5 × (distance − 2 m) (in m/s), capped at 0.6 m/s. It stops within 0.2 m of 2 m.
   - If you're more than 35° off to the side, it turns on the spot first.
   - It never reverses.
5. **Safety.**
   - Not seeing you for 1 s (at least 3 lidar turns): it stops and unlocks. Step back in front of it, within 3 m, to start again.
   - Anything closer than 0.7 m within ±25° ahead: no forward driving.
   - No lidar data for 1 s: it stops.
   - It sends commands 20 times a second, and Ctrl+C sends a stop.

Distances are measured from the lidar, not the rover's centre.

**In the laptop simulation.** Terminal 1:

```bash
ros2 launch ~/RoverFlake2/src/learn_drive/launch/rover_nav_sim.launch.py person:=true
```

Terminal 2:

```bash
python3 ~/RoverFlake2/src/learn_drive/scripts/lidar_follow.py
```

It locks onto the person 2 m ahead. In RViz, click **Publish Point** and then click somewhere: the person walks there at 0.7 m/s, and the rover follows and stops 2 m away. To see where it thinks the person is, use Add → By topic → `/follow/target` → PointStamped. Don't send Nav2 goals at the same time, because both would be driving.

**On the mini rover.** Bring up CAN and the translator as in 4.6, and run the Sweep driver on the rover's computer (`ros2 run learn_drive sweep_driver`). The IMU and GPS aren't needed. Copy the script over from your laptop:

```bash
scp ~/RoverFlake2/src/learn_drive/scripts/lidar_follow.py rv@<rover address>:~/
```

Step 1: **which way the lidar faces.** This is a dry run: it prints what it sees but doesn't drive. Open the view all the way round, and ignore anything closer than 1 m so the rover's body isn't counted:

```bash
python3 ~/lidar_follow.py --ros-args -p dry_run:=true -p view_half_angle_deg:=180 -p min_range:=1.0
```

Stand about 1.5 m straight in front of the rover, with nothing else within 2 m. Read where it says you are. That's either the "Nearest thing it can see: …" line, or, if it has locked on, the "Following: you are … away, …" line:

- **"straight ahead":** the direction is right. Use `lidar_yaw_deg:=0`.
- **"X deg left":** use `lidar_yaw_deg:=-X`.
- **"X deg right":** use `lidar_yaw_deg:=X`.

Rerun with that setting. Then step to the rover's left (your right, if you're facing it). It should say "left". If it says "right", the lidar's left and right are mirrored: restart the Sweep driver with `-p flip:=true` and redo this step.

Step 2: **the cutoff for the rover's body.** Run a dry run with the normal view, plus the `lidar_yaw_deg` from step 1, and nobody in front:

```bash
python3 ~/lidar_follow.py --ros-args -p dry_run:=true -p lidar_yaw_deg:=<from step 1>
```

If "Nearest thing it can see" keeps showing something close at the same spot, it's part of the rover. Raise `min_range` (for example `-p min_range:=0.5`) or narrow the view (for example `-p view_half_angle_deg:=45`) until it doesn't. Then stand in front: it should say "Locked on", and following lines should track you as you move, still without driving.

Step 3: **drive for real.** Keep someone next to the rover, ready to press Ctrl+C. `turn_sign:=-1` is needed because the mini rover currently turns the wrong way (section 6). Use the lowest commands that actually move it for the two minimum speeds:

```bash
python3 ~/lidar_follow.py --ros-args -p lidar_yaw_deg:=<from step 1> -p turn_sign:=-1 \
  -p min_linear_speed:=0.5 -p min_turn_speed:=0.4
```

Walk slowly away: it should follow. Stop: it should stop about 2 m from you. Step to the side: it should turn toward you. Walk off out of view: it should stop and print "Lost you".

| Problem | Fix |
|---|---|
| Turns back and forth around you | Raise `turn_deadband_deg` (for example 12), or lower `max_turn_speed` |
| Says it's driving but doesn't move | Raise `min_linear_speed` / `min_turn_speed` to commands that actually move it |
| Locks onto a chair or table leg | Start with nothing else within 3 m in front |
| Loses you when you're 3 m or more away | Spin the Sweep slower for more readings per turn: restart the driver with `-p motor_hz:=3` |
| Follows too closely or too far | `follow_distance` (metres, from the lidar) |

Main settings (all `-p name:=value`):

| Setting | Default | Meaning |
|---|---|---|
| `dry_run` | false | Print everything, drive nothing |
| `lidar_yaw_deg` | 0 | Which way the lidar's 0° points, relative to the rover's front (+ = left) |
| `view_half_angle_deg` / `min_range` / `max_range` | 60 / 0.35 / 6.0 | What it looks at: ± angle from straight ahead, and the distance range |
| `follow_distance` | 2.0 | Metres to stay away |
| `max_linear_speed` / `min_linear_speed` | 0.6 / 0.0 | Forward command limits (m/s) |
| `max_turn_speed` / `min_turn_speed` | 1.0 / 0.0 | Turn command limits (rad/s) |
| `turn_sign` | 1 | −1 if a left (+) turn command makes the rover turn right |
| `start_max_distance` / `start_half_angle_deg` | 3.0 / 30 | Where you must stand for it to lock on |
| `max_width` | 0.9 | Blobs wider than this aren't people |
| `lost_timeout` | 1.0 | Seconds without seeing you before stopping |
| `stop_distance` | 0.7 | Something this close ahead: no forward driving |
| `cmd_vel_topic` / `scan_topic` | `/cmd_vel` / `/scan` | Topic names |

The Sweep has been losing its readings after a few minutes of running, so keep runs short.

---

## 5. How it works

### 5.1 Topics and frames used

| Topic | Message type | What it carries | Published by |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` forward speed (m/s), `angular.z` turn speed (rad/s, + = left) | Nav2, square script, teleop |
| `/odom` | `nav_msgs/Odometry` | Where the rover thinks it is, and its speed | `fake_rover` (sim), SimFlake. **Nothing on the mini rover yet** |
| `/scan` | `sensor_msgs/LaserScan` | One lidar turn: start angle, angle step, list of distances. NaN = no reading, inf = nothing hit | `fake_lidar`, `sweep_driver` |
| `/imu/data_raw` | `sensor_msgs/Imu` | Rotation speeds and accelerations, no orientation | Phidgets driver |
| `/imu/data` | `sensor_msgs/Imu` | The same plus orientation (heading and tilt) | Madgwick filter |
| `/gps/fix` (sim), `/gnss_fix` (mini rover) | `sensor_msgs/NavSatFix` | Latitude/longitude | SimFlake / the rover's GPS node |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz's goal button | RViz |
| `/plan`, `/lookahead_point` | `nav_msgs/Path`, `PointStamped` | Nav2's path and steering point | Nav2 |
| `/rover_trail`, `/rocks` | `Path`, `MarkerArray` | Display only | `fake_rover`, `fake_lidar` |

Frames follow the ROS convention: x forward, y left, z up.

- `odom` is fixed to the ground where the rover started. Everything in this package runs in `odom`, with no map.
- `base_footprint` is the rover's centre on the ground (Leo model). The team model uses `base_link`.
- `laser` is the lidar. In the simulation it's 0.3 m above `base_footprint`. In real-lidar mode it hangs off `odom`, 0.1 m up, because the lidar sits still on the floor.

### 5.2 Pretend rover (`fake_rover`)

It does the two jobs the real rover's drive code must do for Nav2: obey `/cmd_vel`, and report where it is.

- 50 times a second it moves toward the commanded speeds, limited the way real motors are: max speed 0.8 m/s, max turn 1.5 rad/s, max acceleration 1.0 m/s², max turn acceleration 3.0 rad/s².
- It stops if no command arrives for 0.5 s, like the mini rover.
- It adds up the motion into a position: `heading += turn_rate·dt`, `x += speed·cos(heading)·dt`, `y += speed·sin(heading)·dt`.
- It publishes `/odom`, the TF `odom → base_footprint`, the wheel angles on `/joint_states` (so Leo's wheels spin in RViz, using wheel radius 0.0625 m and wheel separation 0.358 m), and `/rover_trail`.

### 5.3 Pretend lidar (`fake_lidar`)

It knows where the rocks are (circles: x, y, radius in `odom`) and where the rover is (from `/odom`). For each of 360 beams it calculates how far the beam travels before hitting a rock, and publishes that as `/scan` 10 times a second. Range is 0.15–12 m, with 1 cm noise. The default field has about nine rocks plus a ridge across x = 11 m with a gap between y = 0.5 and 2 m. Clicked points add new rocks (radius 0.3 m).

### 5.4 Nav2 setup (`config/rover_nav2.yaml`)

Nav2 is a set of programs that, given a goal, plan a path and drive it while avoiding obstacles. This setup runs **without a map**: everything is measured from the start point (`odom`).

- **Costmaps:** grids of cells, each with a cost from 0 (free) to 254 (blocked).
  - The obstacle layer marks cells where lidar beams hit something, and clears cells that beams pass through. `inf` readings count as clear.
  - The inflation layer adds a keep-away halo that fades with distance (`inflation_radius` 1.0 m, `cost_scaling_factor` 2.0).
  - The global costmap is a 50 × 50 m window at 0.1 m cells, used for planning. The local costmap is a 6 × 6 m window at 0.05 m cells, used for driving. Both are "rolling": they move with the rover.
  - `robot_radius` is 0.34 m.
- **Planner** (NavFn, A* search): finds the cheapest path through the global costmap. `allow_unknown: true` so it can plan through unseen space. `tolerance` is 0.3 m: if the goal itself is blocked, a free spot this close is accepted.
- **Controller** (Regulated Pure Pursuit), 20 times a second:
  - Steers toward a point on the path about 0.8 m ahead (0.4–1.5 m, growing with speed). That point is the lookahead point, or "carrot".
  - Cruises at 0.5 m/s, and slows down in tight turns, near obstacles, and over the last 0.6 m before the goal.
  - If the path starts behind the rover (more than 45° off), it rotates on the spot first, at 1.0 rad/s.
  - Collision checking is on. Reversing is off.
- **Goal checker:** within 0.3 m of the goal counts as arrived; the final direction doesn't matter.
- **Progress checker:** if the rover moves less than 0.5 m in 10 s, it counts as stuck.
- **Behavior server:** recovery moves (spin, back up, wait).
- **BT navigator:** runs the behavior tree, a flowchart re-checked many times per second. It replans about once a second and follows the path. If that fails, it clears the costmaps, spins, waits, backs up, and retries.
- **Lifecycle manager** (`lifecycle_manager_navigation`): starts the four servers in order.

Lessons from tuning:

- `robot_radius` must match the rover's real size, or it gets wedged next to rocks.
- The planner's `tolerance` must not be bigger than the goal checker's `xy_goal_tolerance`. Otherwise a goal placed inside a rock makes it retry forever.
- `max_obstacle_height` must be above the lidar's mount height, or every lidar hit is ignored.
- When a launch crashes, it can leave old nodes running in the background, which show up as duplicate rovers (section 7).

### 5.5 Square drive (`scripts/square_drive.py`)

It only talks to ROS topics, so the same file runs in SimFlake and on the real rover.

- It sends `/cmd_vel` 20 times a second for the whole run, so the mini rover's 0.5 s safety stop never trips.
- **Sides:** it drives forward until `/odom` says it has gone `side_length` (or, with `distance_source:=time`, for side_length ÷ linear_speed seconds). On the way, it steers back toward the side's direction if the heading drifts.
- **Corners:** it turns on the spot until the IMU heading reaches the target.
  - Turn speed = 1.5 × the angle still to go, kept between `min_turn_speed` and `turn_speed`. This is a proportional controller.
  - It stops within `heading_tolerance_deg`.
  - Each corner's target is measured from the starting direction (90°, 180°, 270°, 360°), so a small miss on one corner is made up on the next instead of adding up.
- **If a sensor goes quiet** for `sensor_stale_s`, it holds the rover still, keeps sending stop, and carries on once data returns. It gives up after `sensor_timeout_s`.
- **Ctrl+C** sends a stop.
- At the end it prints the finishing position and direction, and saves the path to `square_<date>_<time>.csv` in the current folder.

| Setting | Default | Meaning |
|---|---|---|
| `side_length` | 2.0 | Metres per side |
| `linear_speed` | 0.3 | Forward speed command (m/s) |
| `turn_speed` / `min_turn_speed` | 0.6 / 0.35 | Fastest and slowest turn command (rad/s) |
| `turn_direction` | `left` | Which way the corners go |
| `heading_tolerance_deg` | 2.0 | How close to 90° counts as done |
| `distance_source` | `odom` | `odom` or `time` |
| `heading_source` | `imu` | `imu`, `odom` or `time` |
| `rate_hz` | 20.0 | Commands per second |
| `log_csv` | True | Save the path to a CSV file |
| `sensor_stale_s` / `sensor_timeout_s` | 0.5 / 15.0 | Pause if a sensor is quiet this long / give up after this long |
| `cmd_vel_topic`, `odom_topic`, `imu_topic`, `gps_topic` | `/cmd_vel`, `/odom`, `/imu/data`, `/gps/fix` | Topic names |

### 5.6 Sweep lidar protocol (what `sweep_driver` does)

- The Sweep talks at 115200 baud in two-letter commands followed by Enter: `IV` (version), `ID` (settings), `MZ` (is the motor steady, `00` = yes), `MS##` (set motor speed), `LR##` (set sample rate: 01 = 500, 02 = 750, 03 = 1000 per second), `DS` (start scanning), `DX` (stop).
- **It sends nothing until it gets `DS`.** That's why generic lidar probes saw zero bytes from it.
- Data comes in 7-byte samples:
  - a byte where bit 0 means "first sample of a new turn" and bits 1–7 are error flags;
  - the angle (2 bytes, ÷16 gives degrees);
  - the distance (2 bytes, in cm);
  - the signal strength (1 byte);
  - a checksum.
- `DS` replying status 12 means the motor isn't steady yet, and 13 means the motor isn't spinning.
- The driver:
  1. stops any old scan;
  2. changes the motor speed and sample rate only if they differ from what's requested;
  3. waits for the motor to be steady, then starts scanning;
  4. collects one full turn, sorts the readings into 1° slots (keeping the nearest reading per slot, NaN where nothing was read), and publishes.

  A new turn is detected by the Sweep's own flag, or by the angle wrapping past 360°. Error-flagged samples still count for detecting new turns. The publisher is "reliable", so RViz and Nav2 both receive it. On errors, it retries every 2 s.
- The USB adapter in the Sweep's cable is an FTDI serial chip.

### 5.7 The mini rover's drive chain

```
/cmd_vel → mini_rover_translator → /can/main/outgoing → generic CAN interface → can0 (USB-CAN adapter) → motor board (STM32) → L298 drivers → 6 motors
```

- **`mini_rover_translator`** (in `arm_hardware_interface`): converts each `/cmd_vel` into the team's CAN drive message (forward m/s, turn in degrees per second). It resends 20 times a second. If `/cmd_vel` goes quiet for 0.5 s, it sends one stop and prints `cmd_vel stale, sent stop`. Keyboard teleop only sends on key presses, so driving with it stutters.
- **Generic CAN interface:** passes messages between `/can/main/*` topics and the `can0` network interface. It needs `can0` up at **500 kbps** first.
- **Motor board firmware** (FirmFlake, `mini_rover_stm32g431cbu6`):
  - CAN runs at 500 kbps, sample point 87.5%.
  - It sends a heartbeat every 1 s, which shows up on `/can/main/heartbeats`.
  - It stops the motors if no command arrives for 500 ms.
  - **Motor power = 40% × (forward ÷ 1.0 ± turn in °/s ÷ 90)**, capped at 40%. Both of those scale numbers are marked "guess, tune" in the code, so a speed command is really a power level, not a real speed.
  - A zero command lets the motors coast rather than brake.
  - Three motors per side.
- **Nothing comes back up the chain.** There are no encoders, so there is no measured speed and no odometry.

---

## 6. Mini rover findings (2026-11-22)

- **Hardware seen on the rover:**
  - `nubs` computer;
  - Phidget IMU (mounted upside down);
  - Emlid Reach M+ GNSS module (the topics `/gnss_fix` and `/base_fix` were present);
  - RealSense D415 depth camera (`/realsense415/...`), which is relevant for measuring the distance to the astronaut;
  - a lidar at the front (its driver wasn't running);
  - USB-CAN adapter;
  - three L298 motor-driver boards;
  - 4-cell LiPo battery (14.8 V);
  - 4-port USB hub.
- **Turning is reversed.** A positive turn command (`angular.z` > 0, which means left in ROS) makes the rover turn **right**. Driving straight is correct, so the left and right sides look swapped, either in the wiring or in which side each motor is assigned to in the firmware.
  - The IMU reports this correctly: a "left" command made the heading go down.
  - This made the square script shake back and forth on the spot. It kept turning the wrong way until the heading was exactly opposite the target, then flipped between left and right every cycle.
  - Workaround: the flip relay (4.6). Teleop turns and Nav2 will also be reversed until it's fixed properly.
- **Minimum commands that move the rover: not measured yet.** A 0.35 rad/s turn command did turn it on the shop floor. The square used 0.6 m/s forward and 1.0–1.3 rad/s turning. The motors are open-loop, so low commands may not move it on grass or rough ground (0.3 m/s is only 12% motor power). Measure the lowest forward and turn commands that move it before tuning Nav2.
- **Square result:** 1 m sides, `linear_speed` 0.6, turns 1.0–1.3 rad/s, 10° tolerance, flip relay running.
  - All four sides and corners finished in 18.5 s.
  - Corners stopped at 80–82° (because of the 10° tolerance).
  - It ended facing −4.8° from its starting direction.
  - The real end position wasn't taped.
- **The IMU is mounted upside down.** This does **not** affect the filtered heading; the filter works out which way is up. It **does** flip the sign of the raw turn rate (`angular_velocity.z`), so before `robot_localization` or Nav2 use the IMU, the mount must be described with a static transform `base_link → imu_link` that includes a 180° roll.
- **The team's `localization_dev` launch turns the compass on** (`use_mag: True`, gain 0.5). The motors' magnetic fields disturb the compass, so for heading use the filter with `use_mag:=false` as in 4.6.

---

## 7. Known issues and fixes

- **CAN: "No buffer space available"** means messages aren't being acknowledged. On CAN, at least one other board must confirm each message, so this means nothing on the bus is answering. Check that the motor board is powered, that CAN High goes to High and Low to Low at both ends, termination, and that both ends are on 500 kbps. Once heartbeats arrive, it's fixed.
- **CAN: "No such device or address" or "Network is down"** means the USB-CAN adapter dropped off or `can0` was switched off. Plug the adapter **straight into the computer, not through the USB hub**, set `can0` up again, and **restart the CAN launch** (it stays attached to the old interface).
- **CAN: `restart-ms` gives "Device doesn't support restart from Bus Off"**: leave it out; this adapter doesn't support it. "Device or resource busy" means you tried to change settings while `can0` is up; take it down first.
- **Topic missing from `ros2 topic list`**: a topic only appears once a running program publishes or listens on it. For example, `/cmd_vel` appears once the translator is running.
- **Duplicate or flickering rovers in the simulation**: old nodes from a crashed launch are still running. Clean up with:

  ```bash
  pkill -f "learn_drive|nav2|rviz2|robot_state_publisher|static_transform_publisher|lifecycle_manager"
  ros2 daemon stop
  ```

- **RViz opens blank** (Fixed Frame `map`): it didn't find the `.rviz` file. Check the path you passed with `-d`.
- **"incompatible QoS" warning on `/scan`**: the listener and publisher disagree on reliable vs best effort. The driver publishes reliable, so this shouldn't happen with it. In RViz, the lidar display is set to Best Effort, which accepts both.
- **Sweep returns no good readings after a few minutes**: everything gets thrown out as "too close" or "nothing came back", even with the lidar untouched. Heat or power is suspected. Let it cool for about 5 minutes, use a USB port with good power, try `-p sample_rate:=500`, and note how long it lasts.
- **SimFlake gotchas:**
  - The Game view's camera is fixed in one spot. To follow the rover, use the Scene tab: select `chassis_link` and press Shift+F.
  - The sim rover keeps doing its last command until it gets a new one.
  - It once fell through the ground. Press Play twice to reset.
  - Its IMU never drifts, so real results will look worse.
  - Data arrives in bursts when the laptop is overloaded.

---

## 8. Next steps and the interface with Pranav (#243)

### Proposed handoff (to be agreed)

Pranav's node publishes where the astronaut is, every time the camera sees them:

- **Topic:** `/astronaut/position`
- **Type:** `geometry_msgs/PointStamped`
- **Frame:** the camera's frame (`camera_link`), in metres with x forward and y left. Not pixels, and not the camera's optical axes (where z is forward).
- **Rate:** 5–10 per second.
- **When the astronaut is lost:** stop publishing. The driving side stops the rover if nothing arrives for 2 s.

The driving side converts each position into the ground frame using TF (camera mount → rover → `odom`) and keeps the Nav2 goal 2 m short of it. That covers the task's "stop within 3 m" rule with margin.

Test the driving side without the camera by faking messages:

```bash
ros2 topic pub -r 5 /astronaut/position geometry_msgs/msg/PointStamped \
  "{header: {frame_id: base_link}, point: {x: 4.0, y: 0.5}}"
```

### Driving-side plan

1. **Follow node.** `lidar_follow.py` (4.7) already does the driving part of following, with the person found by the lidar. Its driving logic only needs "person at this distance and angle", so Pranav's `/astronaut/position` can replace the lidar blob as the input. For following around obstacles, the next version would feed the position to Nav2 instead. Options: Nav2's `follow_point` behavior tree, which replans toward a moving goal about once a second and cuts a set distance (2 m) off the end of the path; or Nav2's Following Server, which has a `desired_distance` setting and searches if it loses the target.
2. **COME:** a single Nav2 goal at the astronaut's last position, 2 m short.
3. **STAY:** cancel the Nav2 goal so nothing publishes `/cmd_vel`. The translator and firmware stop the motors within 0.5 s.
4. **Mini rover odometry:** a node publishing `/odom` and the TF `odom → base_link`, from calibrated speed plus the IMU heading, until the rover has encoders.
5. **Real-rover Nav2 launch:** the simulation launch without `fake_rover` and `fake_lidar`, using the mini rover's real size. Nav2 slows down near goals and obstacles, and the open-loop motors may stall at low commands. Once the lowest commands that move it are measured, this needs either minimum speeds set in Nav2, or a node that raises too-small commands up to the minimum that moves the rover.
6. **Sensors on the rover:** the lidar driver running on `nubs`, plus measured static transforms for the lidar, the IMU (upside down: 180° roll) and the camera.
7. **Team fix:** the reversed turning, so the flip relay can be removed.
