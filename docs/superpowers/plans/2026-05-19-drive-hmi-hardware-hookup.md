# Drive HMI Hardware Hookup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire DriveModule, DrivetrainStopModule, and WheelTelemetryModule HMI panels to the post-merge Phidget position-controller drivetrain so they emit and consume real data.

**Architecture:** Extend `motor_control_node` inside `src/drive_control/` with three new behaviors — an e-stop subscriber/status publisher pair, a `/drivetrain/wheel_states` publisher driven off the existing 10 Hz feedback timer, and a per-wheel state cache. No new packages, no new nodes, no new launch files. `DriveModule` already publishes `/cmd_vel`, which the existing `wheel_speed_node → motor_control_node` chain already consumes — that path is unchanged.

**Tech Stack:** ROS 2 (rclcpp), C++17, Phidget22 SDK, `rover_msgs::WheelStates`, `std_msgs::Bool`, colcon build system.

**Note on testing:** The spec explicitly defers automated unit tests — this is pure ROS plumbing against physical hardware (Phidget motor position controllers) for a placeholder telemetry pass that will be rewritten when richer sensors land. Each task is verified by building, then either bench-testing with `ros2 topic` CLI tools or driving real motors. Frequent commits remain.

**Spec:** `docs/superpowers/specs/2026-05-19-drive-hmi-hardware-hookup-design.md`

---

## File Structure

| File | Action | Purpose |
|---|---|---|
| `src/drive_control/include/motor_control.h` | Modify | Add includes, constants (`WHEEL_TO_PORT`, `TORQUE_PROXY_SCALE`, `STATUS_HEARTBEAT_INTERVAL_MS`), `WheelState` struct, new pub/sub/timer member declarations, new method declarations. |
| `src/drive_control/src/motor_control.cpp` | Modify | Implement: WheelStates publisher (stub → real fields across tasks), e-stop sub/pub/heartbeat, `setVelocity()` gate. |
| `src/drive_control/CMakeLists.txt` | Modify | Add `std_msgs` to `motor_control_node`'s `ament_target_dependencies` (currently only `rclcpp geometry_msgs sensor_msgs rover_msgs`; std_msgs is `find_package`'d at the top but not linked into this target). |

Total expected diff: ~120 lines across the three files.

---

## Task 1: WheelStates publisher scaffold (build + topic shape)

Add the publisher, the per-wheel state cache, and a stub `publishWheelStates()` that emits a fully-formed `WheelStates` message of NaN/false values. Goal of this task: confirm the wiring compiles, links, and publishes the right message shape — *before* we touch any Phidget reads.

**Files:**
- Modify: `src/drive_control/include/motor_control.h`
- Modify: `src/drive_control/src/motor_control.cpp`
- Modify: `src/drive_control/CMakeLists.txt`

- [ ] **Step 1: Add includes, constants, struct, and member declarations to the header**

Edit `src/drive_control/include/motor_control.h`. After the existing `#include "rover_msgs/msg/drive_feedback.hpp"` line, add:

```cpp
#include "rover_msgs/msg/wheel_states.hpp"
#include "std_msgs/msg/bool.hpp"
#include <atomic>
#include <array>
#include <limits>
```

After the existing `#define MOTOR_FAILSAFE_INTERVAL_MS 500` block, add:

```cpp
#define NUM_WHEELS 6
#define STATUS_HEARTBEAT_INTERVAL_MS 1000

// Wheel index → Phidget hub port. WheelStates message documents wheel order as
// FL(0), FR(1), ML(2), MR(3), RL(4), RR(5). The existing motor_control treats
// hub ports {3,4,5} as the left side and {0,1,2} as the right side, so:
constexpr int WHEEL_TO_PORT[NUM_WHEELS] = { 3, 0, 4, 1, 5, 2 };

constexpr float TORQUE_PROXY_SCALE = 5.0f;

struct WheelState {
    double prev_pos = 0.0;
    rclcpp::Time prev_time;
    double commanded_vel = 0.0;
    bool engaged = false;
    bool overheat = false;
    bool valid = false;
    bool has_prev = false;
};
```

Inside the `MotorControlNode` class, in the `private:` section, after the existing publisher/subscription declarations, add:

```cpp
rclcpp::Publisher<rover_msgs::msg::WheelStates>::SharedPtr wheel_states_pub_;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_status_pub_;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
rclcpp::TimerBase::SharedPtr stop_heartbeat_timer_;

std::atomic<bool> remote_stop_{false};
WheelState wheel_state_[NUM_WHEELS];

void onRemoteStop(const std_msgs::msg::Bool::SharedPtr msg);
void publishStopStatus();
void publishWheelStates();
```

- [ ] **Step 2: Add std_msgs to motor_control_node's link dependencies**

Edit `src/drive_control/CMakeLists.txt`. Find:

```cmake
ament_target_dependencies(motor_control_node
  rclcpp
  geometry_msgs
  sensor_msgs
  rover_msgs
)
```

Change to:

```cmake
ament_target_dependencies(motor_control_node
  rclcpp
  geometry_msgs
  sensor_msgs
  rover_msgs
  std_msgs
)
```

- [ ] **Step 3: Add stub `publishWheelStates()` body and create the publisher in `motor_control.cpp`**

Edit `src/drive_control/src/motor_control.cpp`. At the top of the constructor, after the existing `drive_feedback_pub_` creation (around line 27-30), insert:

```cpp
wheel_states_pub_ = this->create_publisher<rover_msgs::msg::WheelStates>(
    "/drivetrain/wheel_states", rclcpp::QoS(10).reliable());
```

At the end of `publishDriveFeedback()` (just before its closing brace, after `drive_feedback_pub_->publish(message);`), insert:

```cpp
publishWheelStates();
```

Then, after the `publishDriveFeedback()` function (before `resetFailsafe()`), add the stub:

```cpp
void MotorControlNode::publishWheelStates() {
    rover_msgs::msg::WheelStates msg;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (int w = 0; w < NUM_WHEELS; w++) {
        msg.speed_rpm[w]     = nan;
        msg.torque_nm[w]     = nan;
        msg.temperature_c[w] = nan;
        msg.power_w[w]       = nan;
        msg.enabled[w]       = false;
    }
    wheel_states_pub_->publish(msg);
}
```

- [ ] **Step 4: Build inside the docker container**

Run (from the host shell):

```bash
docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && cd /RoverFlake2 && colcon build --packages-select drive_control --symlink-install 2>&1"
```

Expected: build succeeds. If you see `undefined reference to std_msgs::msg::Bool` it means Step 2 wasn't applied — the new bool subscription declaration in the header needs std_msgs even though we haven't used it in the .cpp yet (the rclcpp template instantiation pulls it).

If `wheel_states.hpp not found`: confirm `rover_msgs` already declared `WheelStates.msg` in its `CMakeLists.txt` — `ls src/rover_msgs/msg/WheelStates.msg` should show the file. It does (verified in spec).

- [ ] **Step 5: Run the node bench-only and confirm the topic shape**

Open three terminals in the docker container (or use `tmux`/`screen`).

Terminal 1 — start the node (will spend up to ~30 s waiting for Phidget attachment timeouts on bench; that's expected):

```bash
source install/setup.bash
ros2 run drive_control motor_control_node
```

Wait until you see the Phidget attachment errors finish printing.

Terminal 2:

```bash
ros2 topic list | grep drivetrain
```

Expected output includes:

```
/drivetrain/wheel_states
```

Terminal 3:

```bash
ros2 topic echo /drivetrain/wheel_states --once
```

Expected output:

```yaml
speed_rpm: [.nan, .nan, .nan, .nan, .nan, .nan]
torque_nm: [.nan, .nan, .nan, .nan, .nan, .nan]
temperature_c: [.nan, .nan, .nan, .nan, .nan, .nan]
power_w: [.nan, .nan, .nan, .nan, .nan, .nan]
enabled: [false, false, false, false, false, false]
```

If the topic publishes but the arrays are empty or wrong-shaped, re-check `publishWheelStates()` — fixed-size 6 arrays in the .msg map to `std::array<T, 6>` and are pre-sized.

- [ ] **Step 6: Confirm publish rate**

```bash
ros2 topic hz /drivetrain/wheel_states
```

Expected: ~10 Hz (matches `DRIVE_FEEDBACK_PUBLISH_FREQUENCY_MS = 100`). Stop the node (`Ctrl+C` in terminal 1).

- [ ] **Step 7: Commit**

```bash
git add src/drive_control/include/motor_control.h src/drive_control/src/motor_control.cpp src/drive_control/CMakeLists.txt
git commit -m "feat(drive_control): scaffold WheelStates publisher with NaN stub

Publishes a fixed-shape WheelStates message at 10 Hz alongside the
existing drive/feedback. Fields are all NaN/false until subsequent
tasks fill them in from Phidget reads."
```

---

## Task 2: Real `speed_rpm` and `enabled` from position controller

Replace the NaN stubs for `speed_rpm` and `enabled` with values derived from the `applied_velocities[]` array (already maintained at ~20 Hz by `motorControlLoop()`) and `Phidget_getAttached`.

**Why this works without position deltas:** the post-merge motor_control node uses `PhidgetMotorPositionController_*` and explicitly tracks `applied_velocities[i]` (rad/s, post-acceleration-clamp). That's a real, signed velocity — no need to differentiate position. Conversion to rpm is `applied_velocities[port] * 60 / (2π)`.

**Files:**
- Modify: `src/drive_control/src/motor_control.cpp`

- [ ] **Step 1: Replace the `publishWheelStates()` body**

Edit `src/drive_control/src/motor_control.cpp`. Replace the stub `publishWheelStates()` body added in Task 1 with:

```cpp
void MotorControlNode::publishWheelStates() {
    rover_msgs::msg::WheelStates msg;
    const float nan = std::numeric_limits<float>::quiet_NaN();

    for (int w = 0; w < NUM_WHEELS; w++) {
        int port = WHEEL_TO_PORT[w];
        WheelState& ws = wheel_state_[w];

        int attached_int = 0;
        PhidgetReturnCode att_ret = Phidget_getAttached(
            (PhidgetHandle)motors[port], &attached_int);
        ws.engaged = (att_ret == EPHIDGET_OK) && (attached_int != 0);
        ws.valid = ws.engaged;

        float v_rad = static_cast<float>(applied_velocities[port]);
        float rpm = v_rad * 60.0f / (2.0f * static_cast<float>(M_PI));

        msg.speed_rpm[w]     = ws.valid ? rpm : nan;
        msg.torque_nm[w]     = nan;
        msg.temperature_c[w] = nan;
        msg.power_w[w]       = nan;
        msg.enabled[w]       = ws.valid;
    }

    wheel_states_pub_->publish(msg);
}
```

Notes:
- `applied_velocities` is a member of `MotorControlNode` (already declared in the header).
- `M_PI` comes from `<cmath>` which is transitively included by the existing headers; if a build error reports it missing, add `#include <cmath>` at the top of `motor_control.cpp`.
- `WheelState` still has unused fields (`prev_pos`, `prev_time`, `has_prev`, `commanded_vel`) — leave them; a follow-up cleanup task can prune them once Tasks 3 and 4 are done.

- [ ] **Step 2: Build**

```bash
docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && cd /RoverFlake2 && colcon build --packages-select drive_control --symlink-install 2>&1"
```

Expected: build succeeds.

- [ ] **Step 3: Bench-verify shape unchanged**

Run the node bench-only:

```bash
docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && source /RoverFlake2/install/setup.bash && ros2 run drive_control motor_control_node 2>&1"
```

In another shell:

```bash
docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && source /RoverFlake2/install/setup.bash && ros2 topic echo /drivetrain/wheel_states --once 2>&1"
```

Expected on bench (no Phidget): `enabled` all `false`, `speed_rpm` all `.nan`. If running interactively is impractical, skip and trust the build.

- [ ] **Step 4: Commit**

```bash
git add src/drive_control/src/motor_control.cpp
git commit -m "feat(drive_control): derive speed_rpm and enabled from applied_velocities

speed_rpm = applied_velocities[port] * 60 / (2π). enabled reflects
Phidget_getAttached. Torque/temp/power remain NaN — filled in next
task."
```

---

## Task 3: Torque/power placeholder + temperature sentinel

Fill `torque_nm`, `power_w`, and `temperature_c` using the `applied_velocities[]` value already available from Task 2. No `runMotors()` changes — that function no longer exists; the post-merge code uses `setVelocity()` + `motorControlLoop()` which maintains `applied_velocities[]` for us.

**Files:**
- Modify: `src/drive_control/src/motor_control.cpp`

- [ ] **Step 1: Fill torque/power/temperature in `publishWheelStates()`**

In `publishWheelStates()`, find the three NaN assignments inserted by Task 2:

```cpp
        msg.torque_nm[w]     = nan;
        msg.temperature_c[w] = nan;
        msg.power_w[w]       = nan;
```

Replace with:

```cpp
        float torque = v_rad * TORQUE_PROXY_SCALE;
        msg.torque_nm[w]     = ws.valid ? torque : nan;
        msg.power_w[w]       = ws.valid ? (torque * std::abs(v_rad)) : nan;
        msg.temperature_c[w] = ws.valid ? (ws.overheat ? 90.0f : 25.0f) : nan;
```

`v_rad` is the local `float` already computed at the top of the per-wheel loop in Task 2. `ws.overheat` stays `false` for now — the Phidget overheat API call is identified at first on-rover bring-up (Task 6 Step 4); until then `temperature_c` reports `25.0` whenever the channel is attached, which the HMI renders green.

- [ ] **Step 2: Add `<cmath>` include if not already present**

At the top of `src/drive_control/src/motor_control.cpp`, ensure both includes are present:

```cpp
#include <algorithm>
#include <cmath>
```

If `<cmath>` is missing, add it (`std::abs` on `float` needs it).

- [ ] **Step 3: Build**

```bash
docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && cd /RoverFlake2 && colcon build --packages-select drive_control --symlink-install 2>&1"
```

Expected: build succeeds.

- [ ] **Step 4: Bench-verify**

Optional — run the node and `ros2 topic echo /drivetrain/wheel_states --once`. On bench (no Phidget), `enabled` is false so all fields are NaN. Real values appear on-rover.

- [ ] **Step 5: Commit**

```bash
git add src/drive_control/src/motor_control.cpp
git commit -m "feat(drive_control): fill torque/power/temperature placeholders

torque_nm = applied_velocity * TORQUE_PROXY_SCALE (rad/s proxy).
power_w   = torque * |applied_velocity|.
temperature_c = 25C sentinel (90C when overheat flag set; flag stays
false until Phidget API call is identified on bring-up)."
```

---

## Task 4: E-stop subscriber, status publisher, heartbeat, setVelocity gate

Add the bidirectional e-stop path and 1 Hz status heartbeat. Gate `setVelocity()` so commands are silently dropped while stopped. The post-merge code has real `PhidgetMotorPositionController_setEngaged` — we use it for true H-bridge disable on stop.

**Files:**
- Modify: `src/drive_control/src/motor_control.cpp`

- [ ] **Step 1: Create publisher, subscription, and heartbeat timer in the constructor**

In `MotorControlNode::MotorControlNode()`, right after the `wheel_states_pub_` creation from Task 1, add:

```cpp
auto stop_qos = rclcpp::QoS(10).reliable().transient_local();
stop_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/drivetrain/remote_stop_status", stop_qos);

stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/drivetrain/remote_stop", rclcpp::QoS(10).reliable(),
    std::bind(&MotorControlNode::onRemoteStop, this, std::placeholders::_1));

stop_heartbeat_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(STATUS_HEARTBEAT_INTERVAL_MS),
    std::bind(&MotorControlNode::publishStopStatus, this));
```

Then, at the very end of the constructor (after the `motor_control_timer_` creation), publish an initial status so any HMI that connects sees the durable latched value:

```cpp
publishStopStatus();
```

- [ ] **Step 2: Add `onRemoteStop()` and `publishStopStatus()` implementations**

After `publishWheelStates()` in `src/drive_control/src/motor_control.cpp`, add:

```cpp
void MotorControlNode::onRemoteStop(const std_msgs::msg::Bool::SharedPtr msg) {
    bool requested = msg->data;
    bool prev = remote_stop_.exchange(requested);

    if (requested && !prev) {
        // false → true: zero velocity intent, disengage so motors coast.
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (!motors[i]) continue;
            target_velocities[i] = 0.0;
            applied_velocities[i] = 0.0;
            PhidgetMotorPositionController_setEngaged(motors[i], 0);
        }
        RCLCPP_WARN(this->get_logger(), "Drivetrain remote-stop ENGAGED");
    } else if (!requested && prev) {
        // true → false: re-engage. target/applied velocities stay at 0 from
        // the stop transition — operator must move the joystick to re-issue
        // intent.
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (!motors[i]) continue;
            PhidgetMotorPositionController_setEngaged(motors[i], 1);
        }
        RCLCPP_INFO(this->get_logger(), "Drivetrain remote-stop RELEASED");
    }

    publishStopStatus();
}

void MotorControlNode::publishStopStatus() {
    if (!stop_status_pub_) return;
    std_msgs::msg::Bool msg;
    msg.data = remote_stop_.load();
    stop_status_pub_->publish(msg);
}
```

- [ ] **Step 3: Gate `setVelocity()` on `remote_stop_`**

Find `setVelocity()` in `motor_control.cpp`. At the very top of the function body, before `double velocity_rads;`, insert:

```cpp
    if (remote_stop_.load()) return;
```

This silently drops incoming velocity requests while stopped — the joystick can move but nothing reaches the motors.

- [ ] **Step 4: Reflect `remote_stop_` in `WheelStates.enabled`**

In `publishWheelStates()`, change the `enabled` assignment from:

```cpp
        msg.enabled[w]       = ws.valid;
```

to:

```cpp
        msg.enabled[w]       = ws.valid && !remote_stop_.load();
```

- [ ] **Step 5: Build**

```bash
docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && cd /RoverFlake2 && colcon build --packages-select drive_control --symlink-install 2>&1"
```

Expected: build succeeds.

- [ ] **Step 6: Bench-verify e-stop round trip**

Terminal 1:

```bash
source install/setup.bash
ros2 run drive_control motor_control_node
```

Terminal 2 — observe status:

```bash
ros2 topic echo /drivetrain/remote_stop_status
```

Expected immediate first message: `data: false` (initial publish from constructor + 1 Hz heartbeat thereafter).

Terminal 3 — pulse the stop:

```bash
ros2 topic pub --once /drivetrain/remote_stop std_msgs/Bool '{data: true}'
```

Expected: terminal 2 immediately prints `data: true` (transition publish), and continues emitting `data: true` at 1 Hz. Terminal 1 logs `Drivetrain remote-stop ENGAGED`.

Toggle back:

```bash
ros2 topic pub --once /drivetrain/remote_stop std_msgs/Bool '{data: false}'
```

Expected: terminal 2 prints `data: false`, terminal 1 logs `Drivetrain remote-stop RELEASED`.

Stop the node.

- [ ] **Step 7: Commit**

```bash
git add src/drive_control/src/motor_control.cpp
git commit -m "feat(drive_control): wire /drivetrain/remote_stop and status

Subscribes to /drivetrain/remote_stop. On false→true: zero velocity
and disengage all motors. On true→false: re-engage; operator must
re-issue joystick intent. Status published immediately on transition
and on a 1 Hz heartbeat with transient_local QoS so the HMI sees the
current state on connect."
```

---

## Task 5: Bench end-to-end verification with the HMI

No code changes — this task confirms the HMI panels light up correctly with no Phidget hardware connected. Once this passes, you're ready to plug into the rover.

- [ ] **Step 1: Launch motor_control_node in a docker shell**

```bash
docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && source /RoverFlake2/install/setup.bash && ros2 run drive_control motor_control_node 2>&1"
```

Leave it running. Wait until Phidget attachment timeout logs settle.

- [ ] **Step 2: Launch the HMI in a separate docker shell**

```bash
docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && source /RoverFlake2/install/setup.bash && ros2 run rover_hmi_core rover_hmi 2>&1"
```

- [ ] **Step 3: Verify WheelTelemetryModule**

The wheel telemetry panel should show all 6 rows (FL, FR, ML, MR, RL, RR). On bench (no Phidget):

- `Speed (rpm)`: `--` (NaN renders as `--`)
- `Torque (Nm)`: `--`
- `Temp (°C)`: `--`
- `Power (W)`: `--`
- `Enabled`: `● OFF` in red on every row

Status row at the bottom: `Live · Wheel states received`.

If the panel still shows "Waiting for wheel data...", check that `motor_control_node` is actually publishing (`ros2 topic hz /drivetrain/wheel_states` should show ~10 Hz).

- [ ] **Step 4: Verify DrivetrainStopModule**

The drivetrain-stop panel should show `RUNNING` in green with a red `STOP` button.

Click `STOP`. Expected:
- Panel text turns red `STOPPED` and blinks.
- Button label changes to `RELEASE` (green border).
- Warning text appears: `⚠ Drivetrain is remotely stopped`.

Click `RELEASE`. Expected: returns to `RUNNING` / green / red `STOP` button.

In another terminal verify the topic round-trip:

```bash
ros2 topic echo /drivetrain/remote_stop_status
```

Should mirror your clicks within 100 ms.

- [ ] **Step 5: Verify DriveModule (already wired before this work)**

Open the drive-module panel. Move the virtual joystick. In another terminal:

```bash
ros2 topic echo /cmd_vel
```

Expected: Twist messages at 20 Hz reflecting joystick motion. (No motors will spin on bench, but the topic must flow — this proves the cmd_vel path is intact and unaffected by the changes.)

- [ ] **Step 6: Shut down**

Close the HMI and stop `motor_control_node`. Nothing to commit — verification only.

---

## Task 6: On-rover verification + overheat API discovery

This task is the actual hardware bring-up. It produces no code changes by default, but Step 4 includes a follow-up edit pattern if the Phidget overheat API turns out to be readable.

- [ ] **Step 1: Bring up the full drive stack on the rover**

On the rover (or via SSH into the rover docker):

```bash
ros2 launch drive_control drive_the_rover.launch.py
```

Then in a separate shell:

```bash
ros2 run rover_hmi_core rover_hmi
```

- [ ] **Step 2: Verify per-wheel sign and port mapping**

Sit the rover on blocks (no contact with the ground). Open the WheelTelemetry HMI panel.

Move the HMI joystick **forward** briefly. Expected: all 6 `Speed (rpm)` cells go non-zero with the **same sign** (the rover rolls forward; both sides spin forward). If a row shows opposite sign, that wheel is mechanically inverted relative to the others — note it for the wiring fix below.

Move the joystick **right** (pure rotation). Expected: left rows (FL/ML/RL) go one direction, right rows (FR/MR/RR) go the opposite. Confirm `WHEEL_TO_PORT` actually maps each row to the physically correct wheel:

```text
Joystick forward only:  all 6 RPM same sign           ✓ correct
Joystick right turn:    left rows opposite to right   ✓ correct
Any row stays at 0:     that Phidget port is dead     ✗ check hub wiring
Any row shows wrong direction: edit WHEEL_TO_PORT[w]  ✗ swap port indices
```

If the mapping is wrong, edit `WHEEL_TO_PORT` in `src/drive_control/include/motor_control.h`, rebuild, and re-test. Commit the corrected mapping:

```bash
git add src/drive_control/include/motor_control.h
git commit -m "fix(drive_control): correct WHEEL_TO_PORT mapping per chassis wiring"
```

- [ ] **Step 3: Verify the e-stop physical effect**

With the rover still on blocks, move the joystick forward so wheels are spinning. Click HMI `STOP`. Expected:

1. All 6 motors coast (no torque) within ~100 ms.
2. HMI panel turns red `STOPPED` / blinks.
3. WheelTelemetry `Enabled` column flips to `● OFF` red on all 6 rows.
4. Speed_rpm decays toward 0 as the wheels spin down.
5. Continued joystick motion has no effect.

Click `RELEASE`. Verify wheels do **not** automatically start moving — you must re-grab the joystick to resume. This is the safer default per the design.

- [ ] **Step 4: (Optional) Discover and wire the Phidget overheat flag**

While the rover is running, deliberately stall or load a motor to trigger overheating, or simply wait through a long drive until a controller warms up.

If the temp cell on that row stays at `25.0` (green) even when the controller is physically hot, the overheat flag isn't being read. Check the Phidget22 docs for the MotorPositionController channel — likely candidates:

- An `Error` event with `EEPHIDGET_OVERTEMP` (`0x0E` or similar). Subscribed via `Phidget_setOnErrorHandler`.
- A `PhidgetTemperatureSensor` channel on the same hub port.
- `Phidget_getDeviceTemperature` on the channel handle (whole device, not per-channel).

Whichever surfaces, add a one-liner read to `publishWheelStates()` that sets `ws.overheat = <read result>` before the `temperature_c` assignment, then rebuild and verify the cell flips to red `90.0` (HMI color thresholds: ≥70 red).

If no API surfaces overheat cleanly, leave `temperature_c` at the `25.0` sentinel and treat overheat as a future hardware-sensor task. Either way: commit nothing if no code change, or commit the new read if it works:

```bash
git add src/drive_control/src/motor_control.cpp
git commit -m "feat(drive_control): wire Phidget overheat flag into temperature_c sentinel"
```

- [ ] **Step 5: Final sanity sweep**

Confirm the rover does what it always did:

1. Joystick → `/cmd_vel` → `wheel_speed_node` → `motor_control_node` → wheels move. *(unchanged path)*
2. `drive/feedback` topic still publishes at 10 Hz with valid_data/velocities/target_velocities/positions populated. *(unchanged behavior)*
3. New: `/drivetrain/wheel_states` at 10 Hz with sensible values.
4. New: e-stop click coasts motors and is recoverable via release.

No regression in any pre-existing capability is the bar. Nothing to commit if all four pass.

---

## Self-review (already performed)

- **Spec coverage:** Every spec requirement mapped to a task.
  - `WHEEL_TO_PORT` and ordering → Task 1 (header) + Task 6 (on-rover verify)
  - speed_rpm computation → Task 2
  - torque/power/temp placeholders → Task 3
  - enabled = engaged AND valid AND !stopped → Task 2 + Task 4
  - E-stop sub/pub + heartbeat + transient_local → Task 4
  - setVelocity() gate → Task 4
  - PhidgetMotorPositionController_setEngaged(0) on stop, setEngaged(1) on release → Task 4
  - No auto-clear commanded_vel on release → Task 4 (explicit comment)
  - Bench verification matching spec Section 5 → Task 5
  - On-rover verification matching spec Section 5 → Task 6
  - Overheat API discovery deferred → Task 6 Step 4

- **Placeholder scan:** Code in every step. No "TODO" / "TBD" / "add appropriate" / "similar to". The `ws.overheat = false` default is justified inline with a follow-up step rather than left vague.

- **Type consistency:** `publishWheelStates`, `publishStopStatus`, `onRemoteStop`, `WheelState`, `WHEEL_TO_PORT`, `TORQUE_PROXY_SCALE`, `STATUS_HEARTBEAT_INTERVAL_MS`, `NUM_WHEELS`, `remote_stop_`, `wheel_state_[]`, `wheel_states_pub_`, `stop_status_pub_`, `stop_sub_`, `stop_heartbeat_timer_` — all spelled identically across tasks.
