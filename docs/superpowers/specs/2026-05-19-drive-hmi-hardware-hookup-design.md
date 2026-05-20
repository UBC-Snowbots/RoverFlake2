# Drive HMI Hardware Hookup — Design

**Date:** 2026-05-19
**Scope:** Wire the three drive-related HMI modules (`DriveModule`, `DrivetrainStopModule`, `WheelTelemetryModule`) to the real Phidget BLDC drivetrain so they emit and consume real data.

## Background

The `rover_hmi_core` package ships three drive-related plugins:

- **DriveModule** — already publishes `geometry_msgs/Twist` on `/cmd_vel` at 20 Hz. Wired.
- **DrivetrainStopModule** — publishes `std_msgs/Bool` on `/drivetrain/remote_stop`, subscribes to `/drivetrain/remote_stop_status`. **Nothing produces or consumes these topics today.**
- **WheelTelemetryModule** — subscribes to `rover_msgs/WheelStates` on `/drivetrain/wheel_states` (6-element arrays of speed_rpm, torque_nm, temperature_c, power_w, enabled). **Nothing publishes this topic today.**

The existing drivetrain stack lives in `src/drive_control/` and contains three nodes:

- `drive_control_node` — joystick → `/cmd_vel` (used when an external joystick is plugged in; the HMI bypasses this).
- `wheel_speed_node` — `/cmd_vel` → `left_wheel_speeds`/`right_wheel_speeds` (`std_msgs/Float64MultiArray`).
- `motor_control_node` — consumes wheel speeds, drives 6 Phidget BLDC motors via the `phidget22` SDK, publishes `rover_msgs/DriveFeedback` on `drive/feedback` (positions + velocities + target_velocities only).

`motor_control_node` already runs a 10 Hz `feedback_timer_` and a Phidget watchdog reset at 100 Hz.

## Goal

Extend `motor_control_node` so that the three HMI endpoints (`/cmd_vel`, `/drivetrain/remote_stop`, `/drivetrain/wheel_states`) all light up against real hardware. All wiring lives inside `src/drive_control/`. No new packages.

## Hardware reality (per user)

The Phidget BLDC controllers cannot directly report velocity, current, or temperature. They expose:

- Position (revolutions, accumulating)
- Engaged / overheat flags
- Commanded velocity (echo of what we set)

This means several `WheelStates` fields are *placeholders* until richer sensors land. The placeholder semantics are intentional and approved.

## Architecture

```
HMI (rover_hmi)                      drive_control
─────────────────                    ───────────────────────────────
DriveModule          ─/cmd_vel─►     wheel_speed_node ─L/R speeds─►  motor_control_node ─Phidget─► 6× BLDC
                                                                          │
DrivetrainStopModule ─/drivetrain/remote_stop───────────────────────►     │
                     ◄─/drivetrain/remote_stop_status─────────────────────│
                                                                          │
WheelTelemetryModule ◄─/drivetrain/wheel_states───────────────────────────┘
```

Three pieces of behavior are added to `motor_control_node`. Nothing else changes:

1. E-stop subscriber on `/drivetrain/remote_stop` plus status publisher on `/drivetrain/remote_stop_status`.
2. `WheelStates` publisher on `/drivetrain/wheel_states`, driven by the existing 10 Hz `feedback_timer_`.
3. Per-wheel state cache (`WheelState wheel_state_[6]`) holding `prev_pos`, `prev_time`, `commanded_vel`, `engaged`, `overheat`.

The existing `drive/feedback` publisher is preserved unchanged.

## Wheel ordering

`WheelStates` documents order as FL(0), FR(1), ML(2), MR(3), RL(4), RR(5).

`motor_control.cpp` currently treats hub ports `{3,4,5}` as "left" and `{0,1,2}` as "right". We introduce an explicit `WHEEL_TO_PORT[6]` table inside `motor_control.h` so the mapping is a single line to fix when the chassis wiring changes.

```cpp
// Wheel index → Phidget hub port. Edit if chassis wiring changes.
constexpr int WHEEL_TO_PORT[6] = { 3, 0, 4, 1, 5, 2 };  // FL, FR, ML, MR, RL, RR
```

(Exact mapping verified at first bring-up by jogging one wheel at a time and watching which row in the HMI moves.)

## Field semantics for `WheelStates`

| Field | Source | Notes |
|---|---|---|
| `speed_rpm[i]` | `(pos_now - pos_last) / dt * 60` | Phidget position is in revolutions; Δrev/sec × 60 = rpm. Sign preserved. First sample after start emits 0 (no Δt yet). |
| `torque_nm[i]` | `commanded_velocity_[-1..1] * TORQUE_PROXY_SCALE` | Placeholder until real current/torque sensors land. Single constant (e.g. `5.0`) defined in `motor_control.h`. |
| `temperature_c[i]` | Phidget overheat flag | `flag ? 90.0 : 25.0` (sentinel values). HMI already color-codes red ≥70 / yellow ≥50 / green <50, so "hot" lights red and "cool" lights green. If the BLDC channel API does not actually expose an overheat flag at runtime, fall back to `NaN` (HMI shows `--`). Confirm at first bring-up. |
| `power_w[i]` | `torque_nm[i] * abs(commanded_velocity)` | Same proxy; idle → 0. |
| `enabled[i]` | `PhidgetBLDCMotor_getEngaged()` AND `valid_data[i]` from the existing read path | Real value. Goes false when e-stop is active, when attachment failed, or when any Phidget read for that wheel errored this tick. |

## E-stop semantics

State:
- `std::atomic<bool> remote_stop_{false};` member on `MotorControlNode`.
- Subscribe `/drivetrain/remote_stop` (`std_msgs/Bool`, reliable, depth 10).
- Publish `/drivetrain/remote_stop_status` (`std_msgs/Bool`, reliable depth 10, `transient_local` durability so the HMI sees current state on (re)connect).

Behavior:

1. On `/drivetrain/remote_stop` callback: write `remote_stop_ = msg.data`.
   - On transition `false → true`: call `PhidgetBLDCMotor_setTargetVelocity(motors[port], 0)` for all 6 ports, then `PhidgetBLDCMotor_setEngaged(false)` (full coast).
   - On transition `true → false`: call `PhidgetBLDCMotor_setEngaged(true)`. Do **not** auto-clear `commanded_velocity_[i]`; operator must move the joystick to issue intent.
   - Publish status immediately on every transition.
2. Inside `runMotors()`: if `remote_stop_` is true, ignore the incoming velocity and return immediately.
3. New 1 Hz `stop_heartbeat_timer_` re-publishes the current `remote_stop_` value so the HMI gauge stays alive even without transitions.
4. `feedback_timer_` is unchanged: when e-stopped, `WheelStates` still flows; `enabled[i]` reads false; `speed_rpm` decays to 0 as position stops changing; `torque/power` go to 0 because `commanded_velocity = 0`.

## Error handling & edge cases

- **Phidget attachment failures.** Existing logging continues. A motor that never attached gets `enabled[i] = false`, `speed_rpm[i] = 0`, all other fields `NaN`. HMI renders `--`.
- **Position rollover.** Phidget position is `double` and accumulates; no wraparound concern.
- **First-sample dt = 0.** Guard against divide-by-zero; emit 0 for `speed_rpm[i]` on first tick.
- **Stale wheel_speed during stop.** `runMotors()` early-return + Phidget 500 ms watchdog covers it.
- **HMI launches before drive_control.** Both `remote_stop_status` (transient_local) and `wheel_states` arrive on next publish; HMI shows "Waiting for wheel data..." in the interim (existing behavior).
- **drive_control launches before HMI.** 1 Hz status heartbeat ensures HMI catches up within 1 s. Stop subscription is durable.
- **Race on `remote_stop_`.** `std::atomic<bool>` makes the read in `runMotors()` race-free. Phidget calls themselves aren't atomic, but they're idempotent — worst case both setVelocity and setEngaged(false) are issued in quick succession; the engage=false wins on hardware.
- **`drive/feedback` consumers.** Unchanged. `valid_data[i]` continues to reflect whether Phidget reads succeeded; it now also contributes to `WheelStates.enabled[i]`.

### Out of scope

- Per-wheel kill (HMI does not expose it; stop is all-or-nothing).
- Phidget reconnect after mid-run detach (matches existing behavior — `openWaitForAttachment` is constructor-only).
- `WheelStates` transient_local QoS (reliable + depth 10 is enough; WheelTelemetry just waits one tick).
- Replacing the placeholder torque/temp/power semantics with real sensor reads (deferred to a future hardware swap).

## Testing & verification

### Bench (no Phidget hardware)

1. Build workspace; confirm `motor_control_node` compiles.
2. Run `motor_control_node` standalone. Phidget attach calls fail; node must not crash. `/drivetrain/wheel_states` publishes at 10 Hz with `NaN`/`false` everywhere.
3. `ros2 topic echo /drivetrain/wheel_states` — confirm shape: 6-element arrays, correct field order.
4. `ros2 topic pub /drivetrain/remote_stop std_msgs/Bool '{data: true}'` — verify `/drivetrain/remote_stop_status` flips within 100 ms and the 1 Hz heartbeat continues echoing `true`. Toggle back to `false`.
5. Open the HMI: WheelTelemetry table populates (with `--`s for NaN); DrivetrainStop panel reflects CLI-toggled state.

### On rover

1. Launch `drive_the_rover.launch.py` + `rover_hmi`. Joystick at zero: all `speed_rpm ≈ 0`, all `enabled = true`.
2. Nudge HMI joystick forward: three left rows and three right rows show non-zero `speed_rpm` matching direction. If signs are flipped, fix `WHEEL_TO_PORT` (or flip the sign of `commanded_velocity_` per wheel — single line).
3. Click HMI **STOP**: motors coast within ~100 ms; status panel red `STOPPED`; `enabled` column all OFF; joystick ignored.
4. Click **RELEASE**: `enabled` returns to ON; motion only resumes after operator re-issues joystick input.
5. Trigger overheat (artificially or wait for warm-up): the affected row's temp cell turns red.

### Tests not added

No automated unit tests. This is pure ROS plumbing against real hardware; mocking Phidget22 isn't worth the cost for a placeholder telemetry pass that will be rewritten when richer sensors arrive.

## Affected files

- `src/drive_control/include/motor_control.h` — add `WHEEL_TO_PORT`, `WheelState` struct, new pub/sub/timer members, `TORQUE_PROXY_SCALE` constant, header includes for `rover_msgs/msg/wheel_states.hpp` and `std_msgs/msg/bool.hpp`.
- `src/drive_control/src/motor_control.cpp` — implement subscriber callback, status publisher, heartbeat timer, `runMotors()` gate, `WheelStates` packing inside `publishDriveFeedback()`.
- `src/drive_control/CMakeLists.txt` — add `std_msgs` to `motor_control_node`'s `ament_target_dependencies` (already present in package deps); no other CMake changes.

Total expected diff: roughly 100–130 lines across the two source files.
