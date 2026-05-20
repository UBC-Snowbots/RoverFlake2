# Drive HMI Hardware Hookup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire DriveModule, DrivetrainStopModule, and WheelTelemetryModule HMI panels to the existing Phidget BLDC drivetrain so they emit and consume real data.

**Architecture:** Extend `motor_control_node` inside `src/drive_control/` with three new behaviors — an e-stop subscriber/status publisher pair, a `/drivetrain/wheel_states` publisher driven off the existing 10 Hz feedback timer, and a per-wheel state cache. No new packages, no new nodes, no new launch files. `DriveModule` already publishes `/cmd_vel`, which the existing `wheel_speed_node → motor_control_node` chain already consumes — that path is unchanged.

**Tech Stack:** ROS 2 (rclcpp), C++17, Phidget22 SDK, `rover_msgs::WheelStates`, `std_msgs::Bool`, colcon build system.

**Note on testing:** The spec explicitly defers automated unit tests — this is pure ROS plumbing against physical hardware (Phidget BLDC) for a placeholder telemetry pass that will be rewritten when richer sensors land. Each task is verified by building, then either bench-testing with `ros2 topic` CLI tools or driving real motors. Frequent commits remain.

**Spec:** `docs/superpowers/specs/2026-05-19-drive-hmi-hardware-hookup-design.md`

---

## File Structure

| File | Action | Purpose |
|---|---|---|
| `src/drive_control/include/motor_control.h` | Modify | Add includes, constants (`WHEEL_TO_PORT`, `TORQUE_PROXY_SCALE`, `STATUS_HEARTBEAT_INTERVAL_MS`), `WheelState` struct, new pub/sub/timer member declarations, new method declarations. |
| `src/drive_control/src/motor_control.cpp` | Modify | Implement: WheelStates publisher (stub → real fields across tasks), e-stop sub/pub/heartbeat, `runMotors()` gate, commanded-velocity caching. |
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
docker compose exec rover_dev bash -c "cd /workspaces/RoverFlake2 && colcon build --packages-select drive_control --symlink-install"
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

## Task 2: Real `speed_rpm` and `enabled` from Phidget reads

Replace the NaN stubs for `speed_rpm` and `enabled` with derived values from Phidget position deltas and the engaged flag.

**Files:**
- Modify: `src/drive_control/src/motor_control.cpp`

- [ ] **Step 1: Replace the `publishWheelStates()` body**

Edit `src/drive_control/src/motor_control.cpp`. Replace the stub `publishWheelStates()` body added in Task 1 with:

```cpp
void MotorControlNode::publishWheelStates() {
    rover_msgs::msg::WheelStates msg;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    auto now = this->get_clock()->now();

    for (int w = 0; w < NUM_WHEELS; w++) {
        int port = WHEEL_TO_PORT[w];
        WheelState& ws = wheel_state_[w];

        double pos = 0.0;
        PhidgetReturnCode pos_ret = PhidgetBLDCMotor_getPosition(motors[port], &pos);
        bool pos_ok = (pos_ret == EPHIDGET_OK);

        int engaged_int = 0;
        PhidgetReturnCode eng_ret = PhidgetBLDCMotor_getEngaged(motors[port], &engaged_int);
        ws.engaged = (eng_ret == EPHIDGET_OK) && (engaged_int != 0);

        ws.valid = pos_ok;

        float speed_rpm = 0.0f;
        if (pos_ok && ws.has_prev) {
            double dt = (now - ws.prev_time).seconds();
            if (dt > 1e-6) {
                speed_rpm = static_cast<float>((pos - ws.prev_pos) / dt * 60.0);
            }
        }
        if (pos_ok) {
            ws.prev_pos = pos;
            ws.prev_time = now;
            ws.has_prev = true;
        }

        msg.speed_rpm[w]     = pos_ok ? speed_rpm : nan;
        msg.torque_nm[w]     = nan;
        msg.temperature_c[w] = nan;
        msg.power_w[w]       = nan;
        msg.enabled[w]       = ws.valid && ws.engaged;
    }

    wheel_states_pub_->publish(msg);
}
```

- [ ] **Step 2: Build**

```bash
docker compose exec rover_dev bash -c "cd /workspaces/RoverFlake2 && colcon build --packages-select drive_control --symlink-install"
```

Expected: build succeeds.

- [ ] **Step 3: Bench-verify shape unchanged with reads attempted**

Run the node in terminal 1:

```bash
source install/setup.bash
ros2 run drive_control motor_control_node
```

Terminal 2:

```bash
ros2 topic echo /drivetrain/wheel_states --once
```

Expected on bench (no Phidget): `speed_rpm` all `.nan`, `enabled` all `false`, torque/temp/power still `.nan`. The Phidget read failures show up in terminal 1 as logged errors but the node continues and the topic still publishes at 10 Hz.

Stop the node.

- [ ] **Step 4: Commit**

```bash
git add src/drive_control/src/motor_control.cpp
git commit -m "feat(drive_control): derive speed_rpm and enabled from Phidget reads

speed_rpm is computed from position deltas (rev/sec * 60). enabled
reflects PhidgetBLDCMotor_getEngaged AND a successful position read.
Torque/temp/power remain NaN — filled in next task."
```

---

## Task 3: Torque/power placeholder + temperature sentinel

Cache the commanded velocity per wheel inside `runMotors()`, then use it to populate `torque_nm`, `power_w`, and the `temperature_c` sentinel.

**Files:**
- Modify: `src/drive_control/src/motor_control.cpp`

- [ ] **Step 1: Cache commanded velocity inside `runMotors()`**

Edit `src/drive_control/src/motor_control.cpp`. Find `runMotors()`:

```cpp
void MotorControlNode::runMotors(const std::vector<int>& selected_motors, float velocity) {
    PhidgetLog_enable(PHIDGET_LOG_INFO, "phidgetlog.log");
    velocity = std::clamp(velocity, -1.0f, 1.0f);

    for (int motor_index : selected_motors) {
        PhidgetReturnCode ret = PhidgetBLDCMotor_setTargetVelocity(motors[motor_index], velocity);
        if (ret != EPHIDGET_OK) {
            handlePhidgetError(ret, "set target velocity", motor_index);
        }
    }
}
```

Replace with:

```cpp
void MotorControlNode::runMotors(const std::vector<int>& selected_motors, float velocity) {
    PhidgetLog_enable(PHIDGET_LOG_INFO, "phidgetlog.log");
    velocity = std::clamp(velocity, -1.0f, 1.0f);

    for (int motor_index : selected_motors) {
        PhidgetReturnCode ret = PhidgetBLDCMotor_setTargetVelocity(motors[motor_index], velocity);
        if (ret != EPHIDGET_OK) {
            handlePhidgetError(ret, "set target velocity", motor_index);
        }
        for (int w = 0; w < NUM_WHEELS; w++) {
            if (WHEEL_TO_PORT[w] == motor_index) {
                wheel_state_[w].commanded_vel = velocity;
                break;
            }
        }
    }
}
```

- [ ] **Step 2: Fill torque/power/temperature in `publishWheelStates()`**

In `publishWheelStates()`, find the three NaN assignments:

```cpp
        msg.torque_nm[w]     = nan;
        msg.temperature_c[w] = nan;
        msg.power_w[w]       = nan;
```

Replace with:

```cpp
        float cmd_v = static_cast<float>(ws.commanded_vel);
        float torque = cmd_v * TORQUE_PROXY_SCALE;
        msg.torque_nm[w]     = pos_ok ? torque : nan;
        msg.power_w[w]       = pos_ok ? (torque * std::abs(cmd_v)) : nan;
        msg.temperature_c[w] = pos_ok ? (ws.overheat ? 90.0f : 25.0f) : nan;
```

The `ws.overheat` field stays `false` for now — the Phidget overheat API call will be identified at first on-rover bring-up (Task 6, Step 4) and wired in as a follow-up. Until then `temperature_c` reports `25.0` whenever the position read succeeded, which the HMI renders green.

- [ ] **Step 3: Add `<cmath>` include if not already present**

At the top of `src/drive_control/src/motor_control.cpp`, ensure both includes are present (the existing file already has `<algorithm>` for `std::clamp`; we need `<cmath>` for `std::abs` on floats):

```cpp
#include <algorithm>
#include <cmath>
```

If `<cmath>` is missing, add it.

- [ ] **Step 4: Build**

```bash
docker compose exec rover_dev bash -c "cd /workspaces/RoverFlake2 && colcon build --packages-select drive_control --symlink-install"
```

Expected: build succeeds.

- [ ] **Step 5: Bench-verify (no Phidget hardware)**

Run the node, then in another terminal:

```bash
source install/setup.bash
ros2 topic echo /drivetrain/wheel_states --once
```

Expected: still all NaN on bench because position reads all fail without hardware. We'll verify real values on rover in Task 6.

Stop the node.

- [ ] **Step 6: Commit**

```bash
git add src/drive_control/src/motor_control.cpp
git commit -m "feat(drive_control): fill torque/power/temperature placeholders

torque_nm = commanded_vel * TORQUE_PROXY_SCALE.
power_w = torque * |commanded_vel|.
temperature_c = sentinel (25C when valid, 90C when overheat flag set).
Overheat flag stays false until Phidget API call is identified on bring-up."
```

---

## Task 4: E-stop subscriber, status publisher, heartbeat, runMotors gate

Add the bidirectional e-stop path and the 1 Hz status heartbeat. Gate `runMotors()` so commands are silently dropped while stopped.

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

Then, at the very end of the constructor (after the `failsafe_timer_` creation), publish an initial status so any HMI that connects sees the durable latched value:

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
        // false → true: stop everything and coast.
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (!motors[i]) continue;
            PhidgetBLDCMotor_setTargetVelocity(motors[i], 0.0);
            PhidgetBLDCMotor_setEngaged(motors[i], 0);
        }
        for (int w = 0; w < NUM_WHEELS; w++) wheel_state_[w].commanded_vel = 0.0;
        RCLCPP_WARN(this->get_logger(), "Drivetrain remote-stop ENGAGED");
    } else if (!requested && prev) {
        // true → false: re-engage; do NOT auto-clear commanded_vel, operator
        // must re-issue intent.
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (!motors[i]) continue;
            PhidgetBLDCMotor_setEngaged(motors[i], 1);
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

- [ ] **Step 3: Gate `runMotors()` on `remote_stop_`**

At the very top of `runMotors()`, before `PhidgetLog_enable`, insert:

```cpp
    if (remote_stop_.load()) return;
```

- [ ] **Step 4: Reflect `remote_stop_` in `WheelStates.enabled`**

In `publishWheelStates()`, change the `enabled` assignment from:

```cpp
        msg.enabled[w]       = ws.valid && ws.engaged;
```

to:

```cpp
        msg.enabled[w]       = ws.valid && ws.engaged && !remote_stop_.load();
```

- [ ] **Step 5: Build**

```bash
docker compose exec rover_dev bash -c "cd /workspaces/RoverFlake2 && colcon build --packages-select drive_control --symlink-install"
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
docker compose exec rover_dev bash -c "source install/setup.bash && ros2 run drive_control motor_control_node"
```

Leave it running. Wait until Phidget attachment timeout logs settle.

- [ ] **Step 2: Launch the HMI in a separate docker shell**

```bash
docker compose exec rover_dev bash -c "source install/setup.bash && ros2 run rover_hmi_core rover_hmi"
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

If the temp cell on that row stays at `25.0` (green) even when the controller is physically hot, the overheat flag isn't being read. Check the Phidget22 docs for the BLDC channel — likely candidates:

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
  - runMotors() gate → Task 4
  - setEngaged(false) on stop, setEngaged(true) on release → Task 4
  - No auto-clear commanded_vel on release → Task 4 (explicit comment)
  - Bench verification matching spec Section 5 → Task 5
  - On-rover verification matching spec Section 5 → Task 6
  - Overheat API discovery deferred → Task 6 Step 4

- **Placeholder scan:** Code in every step. No "TODO" / "TBD" / "add appropriate" / "similar to". The `ws.overheat = false` default is justified inline with a follow-up step rather than left vague.

- **Type consistency:** `publishWheelStates`, `publishStopStatus`, `onRemoteStop`, `WheelState`, `WHEEL_TO_PORT`, `TORQUE_PROXY_SCALE`, `STATUS_HEARTBEAT_INTERVAL_MS`, `NUM_WHEELS`, `remote_stop_`, `wheel_state_[]`, `wheel_states_pub_`, `stop_status_pub_`, `stop_sub_`, `stop_heartbeat_timer_` — all spelled identically across tasks.
