# Heart hardening + new-HMI integration — design

Approved by Aaron 2026-08-02. Working branch: `worktree-heart_hardening` off
`fix/aaronrhim/drilling_pipeline`; Aaron merges when done.

## Problem

The heart system (per-machine `HeartNode` supervisor in `rover_manager` +
`/heart/request` / `/heart/running_subsystems` control plane) has the right
architecture but a flawed implementation, and the new HMI (`rover_hmi_core`)
doesn't use it at all — it forks `moteus_driver` locally (`DriverProcess`),
which only works on the bench.

Flaws being fixed:

- `online` is a belief: children are never reaped (`waitpid` absent), so a
  crashed subsystem beats `running: true` forever and a wedged flag blocks
  re-start.
- Kill is `killpg(SIGINT)` only — no escalation, no restart primitive.
- Liveness is judged by comparing sender `RCL_SYSTEM_TIME` stamps against the
  receiver clock — measures cross-machine clock skew, not liveness.
- One `HeartRequest` msg serves both intent and state; binary on/off can't
  express crashed/stopping.
- New HMI has no subsystem liveness outside the arm panel's hand-rolled
  staleness watchdog.

Decisions made: harden the custom heart (not a systemd bridge — docker/bench
dev compatibility, single yaml, team knows ROS better than systemd, interface
stays stable so systemd could swap in behind the same topics later). Crash
policy is **report only** — no heart-level auto-restart; `arm_interface`
respawn stays the launch file's job. Old GTK dashboard is not updated; it
retires quietly. Bandwidth frugality is a hard constraint (~30 Mbps link,
cameras dominate): one aggregated status message per heart at 1 Hz ≈ 300 B/s.

## Messages (rover_msgs)

New topic names, no type collision with the old dashboard. `HeartRequest` is
kept (unused by new code) so the old dashboard still compiles; deletion is a
later cleanup.

```
SubsystemCommand.msg            # HMI → hearts on /heart/command
  string subsystem_name
  uint8 action                  # ACTION_START=0, ACTION_STOP=1, ACTION_RESTART=2

SubsystemState.msg
  string name
  uint8 state                   # STOPPED=0, RUNNING=1, STOPPING=2, CRASHED=3
  int32 pid                     # -1 when not running
  int32 exit_code               # valid only in CRASHED (killed-by-signal → 128+sig)
  uint32 uptime_s

HeartStatus.msg                 # hearts → HMI on /heart/status
  std_msgs/Header header
  string host
  SubsystemState[] subsystems   # one aggregated msg per beat per heart
```

## Heart hardening (rover_manager)

`heart.yaml` interface unchanged (subsystem name → shell command). Internals:

- **Reap loop**: 250 ms wall timer, `waitpid(pid, WNOHANG)` per tracked child.
  Reap in STOPPING → STOPPED; unexpected reap → CRASHED + exit code (or
  128+signal). State is derived from observation, never a stored belief.
- **Non-blocking stop escalation**: STOP → `killpg(SIGINT)`, state STOPPING;
  not reaped after 3 s → SIGTERM; +3 s → SIGKILL. Driven by the same timer;
  the heart never sleeps/blocks.
- **Restart**: `pending_restart` flag — stop, then start on reap. Can't race
  escalation.
- **Beats**: 1 Hz aggregated `HeartStatus` + immediate publish on any state
  change (crash visible in <250 ms, not up to 1 s).
- **Start guard**: refused only while observed RUNNING/STOPPING.
- Existing self-kill guard (don't `killpg` own group) is preserved.

## New HMI (rover_hmi_core)

**Subsystems panel** — new `GuiModule` ("Subsystems", section General, hint
right). One group per host, one row per subsystem: name, state chip (grey
STOPPED / green RUNNING / amber STOPPING / red CRASHED + exit code), uptime,
Start/Stop/Restart → `/heart/command`. Heart liveness = arrival time on the
HMI's own steady clock; no `HeartStatus` from a host for >2.5 s → host group
red "HEART OFFLINE". Sender stamps are never used for freshness.

**StaleMonitor helper** — extract the arm panel's stamp-on-arrival + QTimer +
threshold pattern into a small shared class; adopt in Wheel Telemetry, Power
Summary, Science Analysis (stale banner / greyed values instead of frozen
numbers). Zero added traffic.

**DriverProcess** stays as the bench fallback in Motor Telemetry, untouched.
(Known wart, out of scope: its `stop()` blocks the GUI thread ~1 s.)

## PR series (single-concern, dependency order)

1. `rover_msgs`: SubsystemCommand / SubsystemState / HeartStatus
2. `rover_manager`: heart hardening (reap, states, escalation, restart,
   aggregated + event beats)
3. `heart.yaml`: real production subsystem lists — **blocked on Aaron's list**
   (placeholders banana/apple/cucumber/meow go; what belongs on each machine
   besides `arm_interface` and a real drive launch?)
4. `rover_hmi_core`: Subsystems panel
5. `rover_hmi_core`: StaleMonitor + adoption in three panels

## Testing (software-only, in docker; Aaron drives runtime checks)

- Heart: test yaml with dummy subsystems (`sleep 300`, a talker). `kill -9`
  the child → `/heart/status` shows CRASHED exit_code=137 within 250 ms.
  STOP a SIGINT-trapping child → observe SIGINT→SIGTERM→SIGKILL escalation
  timing. START while RUNNING → refused. RESTART → clean stop-then-start.
- Panel: run the HMI against that local heart; pull the heart down → host
  group goes HEART OFFLINE.
- Each PR: `colcon build --packages-select <pkg>` clean in the rover
  container.
