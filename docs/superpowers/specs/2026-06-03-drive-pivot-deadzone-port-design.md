# Drive Pivot-Deadzone Port + Failsafe Bugfix — Design

**Date:** 2026-06-03
**Branch:** `feat/aaronrhim/new_hmi`
**Goal:** Bring the one genuinely-new drive behavior from `feat/emily/more_sophisticated_drive_control` (pivot-deadzone prevention) onto `new_hmi`, and fix a variable-shadowing wart in the failsafe reset — without disturbing the existing HMI integration (`/drivetrain/wheel_states`, `/drivetrain/remote_stop`).

## Background

A cross-branch comparison established that `new_hmi` is **not** behind the Emily drive branches — it already carries the Phidget position controller, the acceleration limit, hysteresis, the inline failsafe reset, the e-stop gate, and the `WheelStates` HMI publisher. A literal `git merge` of either Emily branch would *regress* control quality and silently delete the e-stop guard and the `std_msgs` dependency, because both branches edit the same functions the HMI work added to (`setVelocity`, `motorControlLoop`, `publishDriveFeedback`, the constructor).

Therefore this is a **targeted hand-port of specific hunks**, not a merge.

Source branches:
- `more_sophisticated_drive_control` (C) — byte-identical to `new_hmi` except commit `41fc9b1` "First attempt at preventing pivot deadzone".
- `position-based-drive-control` (B) — older sibling; only used here for the conceptual failsafe-shadowing observation. Nothing ported wholesale. Its odometry work is **deferred** (unfinished; would require a breaking `DriveFeedback.msg` change).

## Scope (approved)

1. **Pivot-deadzone prevention** — ported from C, **replacing** hysteresis.
2. **Failsafe de-shadow** — minimal one-line fix in `motorControlLoop()`.

Out of scope: B's separate failsafe-timer refactor; B's odometry feedback / `DriveFeedback.msg` change.

## Change 1 — `wheel_speed`: pivot-deadzone replaces hysteresis

### Problem being fixed (latent bug in `new_hmi`)
`cmdVelCallback` calls `applyHysteresis(current_wheel_velocities, …)`, but `current_wheel_velocities` is **never sized**. `applyHysteresis` and `driveFeedbackCallback` index `[0..2]` into empty `std::vector`s → out-of-bounds / undefined behavior on the first `/cmd_vel`. Porting C's pure-function deadzone removes this whole path.

### `src/drive_control/include/wheel_speed.h`
- Bump `MOTOR_STOP_THRESHOLD` `0.02 → 0.05`.
- Remove the `driveFeedbackCallback`, `drive_feedback_sub_`, `applyHysteresis`, and `current_wheel_velocities` declarations (the broken feedback path).
- Add `WheelVelocities preventPivotDeadzone(WheelVelocities wheel_velocities);`.
- Drop now-unused includes (`rover_msgs/msg/drive_feedback.hpp`) if nothing else needs them.

### `src/drive_control/src/wheel_speed.cpp`
- Remove the `drive_feedback_sub_` creation in the constructor and the `driveFeedbackCallback` definition.
- In `cmdVelCallback`, replace `applyHysteresis(current_wheel_velocities, wheel_velocities)` with `preventPivotDeadzone(wheel_velocities)`.
- Replace the `applyHysteresis` definition with `preventPivotDeadzone`:

```cpp
WheelVelocities WheelSpeedNode::preventPivotDeadzone(WheelVelocities wheel_velocities) {
    WheelVelocities final_wheel_velocities;
    final_wheel_velocities.left_wheel_velocities.resize(NUM_MOTORS / 2);
    final_wheel_velocities.right_wheel_velocities.resize(NUM_MOTORS / 2);
    for (int i = 0; i < NUM_MOTORS / 2; i++) {
        double left_vel  = wheel_velocities.left_wheel_velocities[i];
        double right_vel = wheel_velocities.right_wheel_velocities[i];
        if (std::abs(left_vel) < MOTOR_STOP_THRESHOLD && std::abs(right_vel) > MOTOR_START_THRESHOLD)
            final_wheel_velocities.left_wheel_velocities[i] = right_vel > 0 ? -MOTOR_STOP_THRESHOLD : MOTOR_STOP_THRESHOLD;
        else
            final_wheel_velocities.left_wheel_velocities[i] = left_vel;
        if (std::abs(right_vel) < MOTOR_STOP_THRESHOLD && std::abs(left_vel) > MOTOR_START_THRESHOLD)
            final_wheel_velocities.right_wheel_velocities[i] = left_vel > 0 ? -MOTOR_STOP_THRESHOLD : MOTOR_STOP_THRESHOLD;
        else
            final_wheel_velocities.right_wheel_velocities[i] = right_vel;
    }
    return final_wheel_velocities;
}
```

### Trade-off
Replaces hysteresis (near-zero jitter suppression) with pivot-deadzone (prevents the rover stalling mid-pivot), exactly as `more_sophisticated` chose. Since the current hysteresis path is UB, this swaps broken-hysteresis for tested-deadzone rather than removing working behavior. `#include <cmath>` is required for `std::abs`.

## Change 2 — `motor_control`: de-shadow the failsafe `ret`

In `motorControlLoop()`:

```cpp
// before
PhidgetReturnCode ret = PhidgetMotorPositionController_resetFailsafe(motors[i]);  // shadows outer `ret`
// after
ret = PhidgetMotorPositionController_resetFailsafe(motors[i]);
```

Behavior is unchanged (the failsafe is still reset inline every loop, matching `more_sophisticated`); this only removes the `-Wshadow` wart. Optionally remove the dead `failsafe_timer_` declaration in the header (never created).

## Verification

No automated unit tests (pure ROS plumbing against hardware, per the existing drive-HMI spec convention). Verify by:
1. `colcon build --packages-select drive_control` succeeds with no shadow warning.
2. Bench test: `ros2 topic echo /drivetrain/wheel_states` while issuing `/cmd_vel`, and confirm a pivot command (`angular.z` only) does not leave one side stuck at zero.

## Not changing
- `motor_control.cpp` control loop math, e-stop, `WheelStates`/`DriveFeedback` publishers.
- Any HMI package.
- `DriveFeedback.msg`.
