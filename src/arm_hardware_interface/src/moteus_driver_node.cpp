#include "moteus_driver_node.h"
#include "axis_5_6_differential.h"
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <thread>
#include <stdexcept>
#include <climits>     // PATH_MAX (realpath)

static constexpr uint32_t ARM_RUN_RATE_MS = (1000u / 100u); // 100 hz
#define LOG_QUEUE_SIZE_MSGS 50
#define SKIP_CALIBRATION
using namespace std::chrono_literals;

namespace {
constexpr float kHomingStepRev = 0.05f;

inline bool stateMachineOwns(AxisState s) {
    return s == AxisState::REQUESTING_HOMING
        || s == AxisState::HOMING
        || s == AxisState::GOING_TO_PRESET_POSITION;
}

// Pure axis->motor for the coupled wrist. Assumes both inputs already
// coherent (same mode, shared limits) — resolveWrist() guarantees that.
// cur_m5/cur_m6 are present motor positions, used to synchronize position moves.
void combine_wrist(const MotorCommand& a5, const MotorCommand& a6,
                   MotorCommand& m5, MotorCommand& m6,
                   float cur_m5, float cur_m6) {
    m5 = a5;  m6 = a6;                          // carry flags + shared limits
    if (!(a5.active || a6.active)) { m5.active = m6.active = false; return; }
    m5.active = m6.active = true;
    if (a5.is_stop || a6.is_stop) { m5.is_stop = m6.is_stop = true; return; }
    m5.is_stop = m6.is_stop = false;

    float o5, o6;  // differential_drive writes float&; cmd fields are double
    if (!std::isnan(a5.position) && !std::isnan(a6.position)) {
        differential_drive((float)a5.position, (float)a6.position, o5, o6);
        m5.position = o5;  m6.position = o6;
        m5.velocity = m6.velocity = NAN;
        // Summed targets travel different distances per motor; scale each
        // motor's profile by its share so both arrive together and the
        // joint-space path stays straight mid-move.
        float d5 = std::fabs(o5 - cur_m5), d6 = std::fabs(o6 - cur_m6);
        float dmax = std::max(d5, d6);
        if (dmax > 1e-4f) {
            m5.max_velocity     *= d5 / dmax;  m6.max_velocity     *= d6 / dmax;
            m5.max_acceleration *= d5 / dmax;  m6.max_acceleration *= d6 / dmax;
        }
    } else {
        differential_drive((float)a5.velocity, (float)a6.velocity, o5, o6);
        // Summed commands reach 2x the shared cap; rescale both together so
        // nothing exceeds the per-frame velocity_limit (firmware winds up
        // position error past it) and the joint-space direction is preserved.
        const float vlim = (float)a5.max_velocity;  // shared by wrist coherence
        const float mag  = std::max(std::fabs(o5), std::fabs(o6));
        if (!std::isnan(vlim) && mag > vlim) {
            o5 *= vlim / mag;  o6 *= vlim / mag;
        }
        m5.velocity = o5;  m6.velocity = o6;
        m5.position = m6.position = NAN;
    }
}
} // namespace

// =============================================================================
// MoteusDriverNode implementation
//
// For protocol details, units, and data field contents see moteus_protocol.h.
// For joint mapping and direction signs see motor_addressing.h.
// For per-motor PID/limits see motor_config.h.
// =============================================================================

static int64_t steadyMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

MoteusDriverNode::MoteusDriverNode() : Node("moteus_driver") {
    // Use a non-singleton transport so we own the only shared_ptr and can truly
    // release the CAN fd when calibration needs exclusive access to the bus.
    // transport_ = mot::TransportRegistry::singleton().make({}).first;
    configs_   = get_arm_configuration();

    for(int i = 0; i < NUM_AXES; i++)
    {
        auto& axis = axes[i];
        axis.homed = false;
        axis.index = i;
        axis.state = AxisState::INIT;

        axis.max_position_rev   = AxisConfig::max_position_rev[i];
        axis.min_position_rev   = AxisConfig::min_position_rev[i];
        axis.homing_speed_revps = AxisConfig::homing_speed_revps[i];
        axis.homing_direction   = AxisConfig::homing_direction[i];
    }

    reInitTransport();

    auto qos     = rclcpp::QoS(5).reliable().durability_volatile();
    auto log_qos = rclcpp::QoS(LOG_QUEUE_SIZE_MSGS).reliable().durability_volatile();

    feedback_pub_  = this->create_publisher<rover_msgs::msg::MoteusArmStatus>("/arm/moteus_feedback", qos);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);
    config_log_pub_  = this->create_publisher<std_msgs::msg::String>("/arm/config_log", log_qos);
    arm_feedback_pub = this->create_publisher<rover_msgs::msg::ArmCommand>("/arm/feedback", qos);

    command_sub_ = this->create_subscription<rover_msgs::msg::ArmCommand>(
        "/arm/command", qos,
        std::bind(&MoteusDriverNode::commandCallback, this, std::placeholders::_1));

    config_update_sub_ = this->create_subscription<rover_msgs::msg::MoteusConfigUpdate>(
        "/arm/config_update", qos,
        std::bind(&MoteusDriverNode::configUpdateCallback, this, std::placeholders::_1));

    config_refresh_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/arm/config_refresh", qos,
        [this](const std_msgs::msg::Empty::SharedPtr) {
            // Enqueue only — run() services one motor at a time so a refresh
            // never freezes the poll loop (the old inline 7-motor sweep here
            // starved the feedback heartbeat and read as "driver offline").
            for (int m = 0; m < NUM_MOTORS; m++) {
                cfg_read_wanted_[m]  = true;
                cfg_read_last_ms_[m] = 0;   // bypass the retry limiter
                cfg_read_fails_[m]   = 0;   // give given-up motors another chance
            }
        });

        #ifndef SKIP_CALIBRATION
        // configureMotors();
        #else 
        RCLCPP_WARN(this->get_logger(), "SKIPPING MOTOR CALIBRATIONS");
        #endif

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&MoteusDriverNode::run, this));

    RCLCPP_INFO(this->get_logger(),
        "Moteus driver started: polling %d motors at 100 Hz", NUM_MOTORS);
}

// ---------------------------------------------------------------------------
// Startup configuration — pushes MotorConfig values to firmware
// ---------------------------------------------------------------------------

void MoteusDriverNode::publishLog(const std::string& msg) {
    std_msgs::msg::String m;
    m.data = msg;
    config_log_pub_->publish(m);
}

void MoteusDriverNode::configureMotors() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        configureMotor(i + 1, *controllers_[i]);

        auto& c = configs_[i];
        publishLog(std::string("# Motor ") + std::to_string(i + 1)
            + " (" + ARM_JOINTS[i].hardware_name + ") configured:"
            + " kp=" + c.format(c.kp)
            + " kd=" + c.format(c.kd)
            + " max_current=" + c.format(c.max_current_A) + "A"
            + " pos=[" + c.format(c.position_min) + ", " + c.format(c.position_max) + "]rev"
            + " max_vel=" + c.format(c.max_velocity) + "rev/s"
            + " gear=" + c.format(c.gear_reduction));
    }
}

void MoteusDriverNode::configureMotor(int motor_id, mot::Controller& ctrl) {
    auto maybe_state = ctrl.SetQuery();
    if (!maybe_state) {
        RCLCPP_WARN(this->get_logger(),
            "Motor %d (%s) NOT CONNECTED. Skipping config.",
            motor_id, ARM_JOINTS[motor_id - 1].hardware_name);
        publishLog("# Motor " + std::to_string(motor_id)
            + " (" + ARM_JOINTS[motor_id - 1].hardware_name + ") NOT CONNECTED — skipped");
        return;
    }

    RCLCPP_INFO(this->get_logger(),
        "Motor %d (%s) detected. Pushing config...",
        motor_id, ARM_JOINTS[motor_id - 1].hardware_name);

    for (const auto& [reg, val] : configs_[motor_id - 1].get_configs()) {
        auto reply = ctrl.DiagnosticCommand("conf set " + reg + " " + val);
        RCLCPP_INFO(this->get_logger(), "[%d] %s = %s (reply: %s)",
            motor_id, reg.c_str(), val.c_str(), reply.c_str());
    }
}

// ---------------------------------------------------------------------------
// Command callback — ROS topic → pending_cmds_[]
// (See arm_commands.h for command code definitions and MotorCommand struct.)
// ---------------------------------------------------------------------------

void MoteusDriverNode::commandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    char cmd_type = msg->cmd_type;

    const bool motor_space = (msg->cmd_value == CMD_SPACE_MOTOR);

    if (cmd_type == CMD_STOP) {
        // positions[] empty = stop all (legacy); else non-NaN entries select.
        const bool masked = !msg->positions.empty();
        for (int a = 0; a < NUM_AXES; a++) {
            if (masked && (a >= (int)msg->positions.size()
                           || std::isnan(msg->positions[a]))) continue;
            pending_axis_cmds_[a].active = true;
            pending_axis_cmds_[a].is_stop = true;
            pending_axis_cmds_[a].motor_space = motor_space;
        }
        RCLCPP_INFO(this->get_logger(), "Command: STOP %s", masked ? "selected" : "ALL");
        return;
    }

    if (cmd_type == CMD_ABS_POS) {
        for (int a = 0; a < NUM_AXES; a++) {
            double pos = (a < (int)msg->positions.size())  ? msg->positions[a]  : NAN;
            double vd  = (a < (int)msg->velocities.size()) ? msg->velocities[a] : NAN;
            if (std::isnan(pos) && std::isnan(vd)) continue;
            auto& p = pending_axis_cmds_[a];
            p.active = true; p.is_stop = false; p.is_zero = false;
            p.motor_space = motor_space;
            p.position = pos;
            const double cap = (double)AxisConfig::max_running_speed[a];
            if (std::isnan(pos)) {
                // d pos nan v nan — velocities[] IS the velocity
                p.velocity = std::clamp(degreesToRevolution(vd), -cap, cap);
                p.max_velocity = cap;
            } else {
                // d pos p v nan — velocities[] is the travel speed, stop at target
                p.velocity = 0.0;
                p.max_velocity = std::isnan(vd) ? cap
                    : std::min(degreesToRevolution(std::fabs(vd)), cap);
            }
            p.max_torque = NAN;
        }
        return;
    }

    if (cmd_type == CMD_ABS_VEL) {
        for (int a = 0; a < NUM_AXES; a++) {
            
            double vd = (a < (int)msg->velocities.size()) ? msg->velocities[a] : NAN;
            if (std::isnan(vd)) continue; // Was breaking?
            auto& p = pending_axis_cmds_[a];
            p.active = true; p.is_stop = false; p.is_zero = false;
            p.motor_space = motor_space;
            p.position = NAN;
            // Clamp INTO the per-frame velocity_limit we send alongside —
            // commanding past it makes the firmware chase an impossible
            // reference: unbounded position error, pinned at the current cap.
            const double cap = AxisConfig::max_running_speed[a];
            p.velocity = std::clamp(degreesToRevolution(vd), -cap, cap);
            p.max_velocity = cap;
            p.max_torque = NAN;
        }
        return;
    }

    if (cmd_type == CMD_ZERO) {   // motor-space diagnostic, bypasses the axis funnel
        for (int m = 0; m < NUM_MOTORS; m++) {
            double flag = (m < (int)msg->positions.size()) ? msg->positions[m] : NAN;
            if (!std::isnan(flag)) motor_zero_req_[m] = true;
        }
        return;
    }

    if (cmd_type == CMD_HOME) {
        if (msg->cmd_value >= 0 && msg->cmd_value < NUM_AXES) { home_axis(msg->cmd_value); return; }
        else if (msg->cmd_value == HOME_VALUE_ALL_AXES_EXCEPT_EE) {
            for (int a = 0; a < NUM_AXES; a++)
                if (a != AXIS_EE_INDEX) home_axis(a);   // gate below denies switchless axes
            return;
        }
        else if (msg->cmd_value == HOME_VALUE_SELECTED) {
            for (double v : msg->positions)
                if (!std::isnan(v) && v >= 0 && v < NUM_AXES) home_axis((uint8_t)v);
            return;
        }
        else if (msg->cmd_value == HOME_VALUE_ALL_AXES_AND_EE)    { /* EE TODO */ return; }
        RCLCPP_WARN(this->get_logger(), "Unknown home command value: %i", msg->cmd_value);
        return;
    }
    RCLCPP_WARN(this->get_logger(), "Unknown cmd_type: '%c' (%d)", cmd_type, (int)cmd_type);
}

// Zero position of a motor.
void MoteusDriverNode::zero_position(uint8_t index)
{
    int i = static_cast<int>(index);
    if (i >= (int)controllers_.size()) return;

    controllers_[i]->DiagnosticCommand("d exact 0");
    RCLCPP_INFO(this->get_logger(),
    "Motor %d (%s) position set to zero (d exact 0)", i + 1, ARM_JOINTS[i].hardware_name);
    publishLog("# Motor " + std::to_string(i + 1)
        + " (" + ARM_JOINTS[i].hardware_name + ") zeroed");
}

void MoteusDriverNode::set_position(uint8_t index, float position_revs)
{
  int i = static_cast<int>(index);
  if (i >= (int)controllers_.size()) return;

  char cmd[32];
  std::snprintf(cmd, sizeof(cmd), "d exact %f", position_revs);
  controllers_[i]->DiagnosticCommand(cmd);
}

void MoteusDriverNode::home_axis(uint8_t index)
{
    // Just set axis state to request homing
    this->axes[index].state = AxisState::REQUESTING_HOMING;
}

// ---------------------------------------------------------------------------
// Run loop — runs at 100 Hz
//
// Step 1: merge pending_cmds_ into active_cmds_  (mutex-protected, O(N))
// Step 2: build one CAN frame per motor           (see moteus_protocol.h)
// Step 3: BlockingCycle — send all frames, wait for all replies
// Step 4: decode replies into telem_[]            (see arm_telemetry.h)
// Step 5: safety checks                           (checkAlerts)
// Step 6: publish /arm/moteus_feedback and /joint_states
// ---------------------------------------------------------------------------

// Pending commands from ros -> merge into active commands, but are rejected if there is homing and whatnot.
// Homing is then Checked. 
void MoteusDriverNode::run() {
    if (!transport_) {
        // Retry detection every ~2 s so plugging in after launch still works.
        static int no_dev_ticks = 0;
        if (++no_dev_ticks >= 200) { no_dev_ticks = 0; reInitTransport(); }
        // No fdcanusb — still publish so the HMI can show the link state.
        // Config must be NaN, not default zeros: zeros render as confident
        // "kp=0" readings in the HMI (audit finding).
        rover_msgs::msg::MoteusArmStatus msg;
        msg.status.resize(NUM_AXES);
        msg.config.resize(NUM_AXES);
        for (auto& fc : msg.config) {
            fc.max_acceleration = fc.max_velocity = fc.min_position =
            fc.max_position = fc.kp = fc.ki = fc.kd = fc.max_current_amps =
            fc.max_voltage_volts = fc.max_power_watts = fc.cmd_timeout_s =
            fc.gear_reduction = NAN;
        }
        msg.limit_switches.resize(NUM_AXES);
        msg.can_device = "";
        msg.motors_replying = 0;
        feedback_pub_->publish(msg);
        return;
    }

    // ── Stage 0: fresh axis_cmds_ every cycle — FULL reset, not just active:
    //    a lingering is_stop on the idle wrist axis would make the coherence
    //    stage turn every later wrist command back into a stop.
    for (auto& c : axis_cmds_) c = MotorCommand{};

    // ── Stage 1: intake — ROS -> axis_cmds_ (only for axes the SM doesn't own)
    // A stop always lands, per axis: it aborts that axis's homing if the SM
    // owns it. A stop-all from the wire arrives as a stop on every axis.
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        for (int a = 0; a < NUM_AXES; a++) {
            auto& p = pending_axis_cmds_[a];
            if (!p.active) continue;
            if (p.is_stop) {
                axis_cmds_[a] = p;
                if (stateMachineOwns(axes[a].state)) axes[a].state = AxisState::RUNNING_OK; // abort homing
            }
            else if (!stateMachineOwns(axes[a].state))  axis_cmds_[a] = p;
        }
        pending_axis_cmds_ = {};
    }

    // ── Stage 2: state machine — homing / preset write axis_cmds_ (axis space)
    for (auto& ax : axes) {
        const int i = ax.index;
        switch (ax.state) {
        case AxisState::REQUESTING_HOMING:
            if (ax.limit_switch) {
                RCLCPP_INFO(this->get_logger(), "Axis %d switch stuck/pressed. Homing denied", i + 1);
                ax.state = AxisState::ERROR;
            } else if (i == AXIS_6_INDEX) {
                RCLCPP_WARN(this->get_logger(), "Axis 6 homing not implemented (runs relative)");
                ax.state = AxisState::RUNNING_OK;
            } else {
                RCLCPP_INFO(this->get_logger(), "Axis %d switch healthy. Homing accepted", i + 1);
                if (i != AXIS_5_INDEX)  // wrist is continuous — no seed needed
                    set_position(i, AxisConfig::max_position_rev[i] - 0.01f);
                ax.state = AxisState::HOMING;
            }
            break;

        case AxisState::HOMING:
            if (ax.limit_switch) {
                RCLCPP_INFO(this->get_logger(), "Axis %d homed", i + 1);
                if (i == AXIS_5_INDEX) { zero_position(MOTOR_5_INDEX); zero_position(MOTOR_6_INDEX); }
                else         { zero_position(i); }
                ax.homed = true;   // counter now switch-referenced → soft limits armed
                ax.state = AxisState::GOING_TO_PRESET_POSITION;
            } else {  // creep toward switch in AXIS space; wrist's other axis is held in Stage 3
                auto& c = axis_cmds_[i];
                c.active = true; c.is_stop = false; c.is_zero = false;
                c.position = axes[i].position + kHomingStepRev * ax.homing_direction;
                c.velocity = NAN;
                c.max_velocity = AxisConfig::homing_speed_revps[i];
                c.max_acceleration = 0.1f; c.max_torque = NAN;
            }
            break;

        case AxisState::GOING_TO_PRESET_POSITION: {
            auto& c = axis_cmds_[i];
            c.active = true; c.is_stop = false; c.is_zero = false;
            c.position = AxisConfig::idle_position[i];  // axis space
            c.velocity = NAN;
            c.max_velocity = 0.1f; c.max_acceleration = 0.1f; c.max_torque = NAN;
            if (std::fabs(axes[i].position - AxisConfig::idle_position[i]) < 0.01f)
            {
                ax.state = AxisState::RUNNING_OK;
                if (i == AXIS_5_INDEX) axes[AXIS_6_INDEX].state = AxisState::RUNNING_OK; // axis 6 usable (relative), unhomed
            }
            break;
        }
        default:
            break;
        }
    }

    // ── Stage 2.5: soft travel limits (axis space) ──────────────────────────
    // Mirror of the sim's clamp: position targets bound to
    // [min_position_rev, max_position_rev] and velocity commands that would
    // drive past a bound become a position-hold AT the bound (not a stop —
    // limp at a soft limit would drop a gravity-loaded joint).
    //
    // Only enforced on HOMED axes: before homing the counter zero is wherever
    // the arm booted, and clamping against that fabricates walls in random
    // places. CMD_ZERO does not arm the limits — only a real switch home does.
    // Homing itself is exempt via stateMachineOwns (it legitimately drives
    // toward and past 0).
    for (int a = 0; a < NUM_AXES; a++) {
        if (!axes[a].homed || stateMachineOwns(axes[a].state)) continue;
        auto& c = axis_cmds_[a];
        if (!c.active || c.is_stop || c.is_zero) continue;
        const double lo = AxisConfig::min_position_rev[a];
        const double hi = AxisConfig::max_position_rev[a];

        if (!std::isnan(c.position)) {
            const double clamped = std::clamp(c.position, lo, hi);
            if (clamped != c.position) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Axis %d position %.3f clamped to soft limit %.3f",
                    a + 1, c.position, clamped);
                c.position = clamped;
            }
        } else if (!std::isnan(c.velocity) && c.velocity != 0.0) {
            const double pos = axes[a].position;
            const bool at_lo = pos <= lo && c.velocity < 0.0;
            const bool at_hi = pos >= hi && c.velocity > 0.0;
            if (at_lo || at_hi) {
                c.position = at_lo ? lo : hi;   // hold the bound
                c.velocity = 0.0;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Axis %d velocity into soft limit %s — holding %.3f",
                    a + 1, at_lo ? "MIN" : "MAX", c.position);
            }
        }
    }

    // ── Stage 3: wrist coherence — make axes 4&5 same mode + shared limits ──
    // Physically the two motors always move as a pair, so one shared vel/accel.
    // Motor-space commands (bench use) bypass coherence + transform entirely.
    const bool raw_wrist =
        (axis_cmds_[AXIS_5_INDEX].active && axis_cmds_[AXIS_5_INDEX].motor_space)
     || (axis_cmds_[AXIS_6_INDEX].active && axis_cmds_[AXIS_6_INDEX].motor_space);
    {
        auto& a5 = axis_cmds_[AXIS_5_INDEX];
        auto& a6 = axis_cmds_[AXIS_6_INDEX];
        if (!raw_wrist && (a5.active || a6.active)) {
            if (a5.is_stop || a6.is_stop) {
                a5.active = a6.active = true; 
                a5.is_stop = a6.is_stop = true;
            } else {
                bool posMode = (a5.active && !std::isnan(a5.position))
                            || (a6.active && !std::isnan(a6.position));
                // limits come from whichever axis is actually driving
                const auto& drv = (a5.active && (posMode ? !std::isnan(a5.position)
                                                         : !std::isnan(a5.velocity))) ? a5 : a6;
                float vlim = std::isnan(drv.max_velocity)     ? 0.1f : drv.max_velocity;
                float alim = std::isnan(drv.max_acceleration) ? 0.1f : drv.max_acceleration;

                auto holdPos = [&](auto& c, int ax){ c.active=true; c.is_stop=false; c.is_zero=false;
                                                     c.position = axes[ax].position; c.velocity = NAN; };
                auto holdVel = [&](auto& c){ c.active=true; c.is_stop=false; c.is_zero=false;
                                             c.position = NAN; c.velocity = 0.0f; };
                if (posMode) {
                    if (!a5.active || std::isnan(a5.position)) holdPos(a5, AXIS_5_INDEX);
                    if (!a6.active || std::isnan(a6.position)) holdPos(a6, AXIS_6_INDEX);
                    a5.velocity = a6.velocity = NAN;
                } else {
                    if (!a5.active || std::isnan(a5.velocity)) holdVel(a5);
                    if (!a6.active || std::isnan(a6.velocity)) holdVel(a6);
                    a5.position = a6.position = NAN;
                }
                a5.active = a6.active = true; a5.is_stop = a6.is_stop = false;
                a5.max_velocity = a6.max_velocity = vlim;
                a5.max_acceleration = a6.max_acceleration = alim;
                a5.max_torque = a6.max_torque = NAN;
            }
        }
    }

    // ── Stage 4: THE transform — axis_cmds_ -> motor_cmds_ (only crossing point)
    for (int m = 0; m < NUM_MOTORS; m++)
        if (m != AXIS_5_INDEX && m != AXIS_6_INDEX)
            motor_cmds_[m] = axis_cmds_[m];   // straight-through
    if (raw_wrist) {
        motor_cmds_[AXIS_5_INDEX] = axis_cmds_[AXIS_5_INDEX];   // raw motor commands
        motor_cmds_[AXIS_6_INDEX] = axis_cmds_[AXIS_6_INDEX];
    } else {
        combine_wrist(axis_cmds_[AXIS_5_INDEX], axis_cmds_[AXIS_6_INDEX],
                      motor_cmds_[AXIS_5_INDEX], motor_cmds_[AXIS_6_INDEX],
                      telem_[AXIS_5_INDEX].position, telem_[AXIS_6_INDEX].position);
        if (motor_cmds_[AXIS_5_INDEX].active)
            RCLCPP_INFO(this->get_logger(), "Axis 5/6 -> motor space  VEL: %0.3f %0.3f -> %0.3f %0.3f  POS: %0.3f %0.3f -> %0.3f %0.3f",
                axis_cmds_[AXIS_5_INDEX].velocity, axis_cmds_[AXIS_6_INDEX].velocity, motor_cmds_[AXIS_5_INDEX].velocity, motor_cmds_[AXIS_6_INDEX].velocity,
                axis_cmds_[AXIS_5_INDEX].position, axis_cmds_[AXIS_6_INDEX].position, motor_cmds_[AXIS_5_INDEX].position, motor_cmds_[AXIS_6_INDEX].position);
    }
    // ── Stage 5: motor-space zero side-channel (diagnostic, no frame) ───────
    for (int m = 0; m < NUM_MOTORS; m++) {
        if (motor_zero_req_[m]) { zero_position(m); motor_zero_req_[m] = false;
                                  motor_cmds_[m].active = false; }
    }

    // ── Stage 6: frames — built ONLY from motor_cmds_ ───────────────────────
    std::vector<mot::CanFdFrame> frames;
    for (int m = 0; m < NUM_MOTORS; m++) {
        auto& c = motor_cmds_[m];
        if      (c.active && c.is_stop) frames.push_back(MoteusProtocol::makeStopFrame(*controllers_[m]));
        else if (c.active)              frames.push_back(MoteusProtocol::makePositionFrame(
                                            *controllers_[m], c.position, c.velocity,
                                            c.max_velocity, c.max_acceleration));
        else                            frames.push_back(MoteusProtocol::makeQueryFrame(*controllers_[m]));
    }

    // ── Stage 7: bus ────────────────────────────────────────────────────────
    std::vector<mot::CanFdFrame> replies;
    transport_->BlockingCycle(frames.data(), frames.size(), &replies);
    rover_msgs::msg::MoteusArmStatus moteus_ros_msg;
    rover_msgs::msg::ArmCommand arm_feedback_msg;
    arm_feedback_msg.positions.resize(NUM_AXES);
    moteus_ros_msg.status.resize(NUM_AXES);
    moteus_ros_msg.config.resize(NUM_AXES);
    moteus_ros_msg.limit_switches.resize(NUM_AXES);
    // ── Stage 8: decode -> telem_, motor->axis (inverse for wrist) ──────────
    for (auto& t : telem_) t.connected = false;
    int motors_replying = 0;
    for (const auto& frame : replies) {
        int id = frame.source;
        if (id < 1 || id > NUM_MOTORS) continue; 
        auto r = MoteusProtocol::parseReply(frame);
        auto& t = telem_[id - 1];
        t.position=r.position; t.velocity=r.velocity; t.torque=r.torque; t.voltage=r.voltage;
        t.temperature=r.temperature;
        t.q_current = std::isnan(r.q_current)?0.0f:(float)r.q_current;
        t.power     = std::isnan(r.power)?0.0f:(float)r.power;
        t.mode=(int)r.mode; t.fault=(int)r.fault;
        if (!t.connected) motors_replying++;
        t.connected=true;
#ifdef DEBUG_LIMIT_SWITCH_RAW_REPLY
        {
            static int prev_gpio[NUM_MOTORS] = {-1, -1, -1, -1, -1, -1, -1};
            const int raw = (int)r.aux2_gpio;
            if (raw != prev_gpio[id - 1]) {
                RCLCPP_INFO(this->get_logger(), "M%d aux2_gpio raw=0x%02x", id, raw);
                prev_gpio[id - 1] = raw;
            }
        }
#endif
        // NOTE: motor-indexed lookup into axis-indexed config — valid only while
        // switch-equipped axes (0-3) map 1:1 to motors; wrist/EE entries are false.
        const bool sw_raw = (r.aux2_gpio & AxisConfig::limit_switch_mask[id - 1]) != 0;
        t.limit_switch = AxisConfig::has_limit_switch[id - 1]
                      && (sw_raw != AxisConfig::limit_switch_inverted[id - 1]);
        axes[id - 1].limit_switch = t.limit_switch;
        if (id - 1 != AXIS_5_INDEX && id - 1 != AXIS_6_INDEX) {
            axes[id - 1].position = t.position;
        } else if (id - 1 == AXIS_6_INDEX) {  // both wrist replies in — invert together
            float a5p, a6p;
            differential_drive_inverse(telem_[AXIS_5_INDEX].position, telem_[AXIS_6_INDEX].position, a5p, a6p);
            axes[AXIS_5_INDEX].position = a5p; axes[AXIS_6_INDEX].position = a6p;
        }
        arm_feedback_msg.positions[id - 1] = axes[id -1].position;
    }

    // Publish last-known telemetry for EVERY motor (telem_ persists between
    // cycles): a motor that misses one reply keeps its previous values with
    // connected=false, instead of collapsing to zeros — which drew square
    // waves in the HMI plots whenever the bus flapped.  des_* mirror what was
    // actually framed this cycle (NaN when only queried).
    for (int m = 0; m < NUM_MOTORS; m++) {
        const auto& t = telem_[m];
        auto& s = moteus_ros_msg.status[m];
        s.connected            = t.connected;
        s.curr_position        = t.position;
        s.curr_velocity        = t.velocity;
        s.curr_torque          = t.torque;
        s.curr_current_amps    = t.q_current;
        s.curr_power_watts     = t.power;
        s.curr_voltage_volts   = t.voltage;
        s.driver_temp_degreesc = t.temperature;
        s.moteus_mode          = static_cast<int16_t>(t.mode);
        s.moteus_fault         = static_cast<int16_t>(t.fault);
        const auto& c = motor_cmds_[m];
        const bool commanding = c.active && !c.is_stop;
        s.des_position = commanding ? c.position : NAN;
        s.des_velocity = commanding ? c.velocity : NAN;
        moteus_ros_msg.limit_switches[m] = t.limit_switch;

        // Actual controller config (conf get results); NaN = never read
        auto& fc = moteus_ros_msg.config[m];
        if (flash_cfg_valid_[m]) {
            const auto& f = flash_cfg_[m];
            auto g = [&f](const char* k) {
                auto it = f.find(k); return it != f.end() ? it->second : NAN; };
            fc.max_acceleration  = g("servo.default_accel_limit");
            fc.max_velocity      = g("servo.default_velocity_limit");
            fc.min_position      = g("servopos.position_min");
            fc.max_position      = g("servopos.position_max");
            fc.kp                = g("servo.pid_position.kp");
            fc.ki                = g("servo.pid_position.ki");
            fc.kd                = g("servo.pid_position.kd");
            fc.max_current_amps  = g("servo.max_current_A");
            fc.max_voltage_volts = g("servo.max_voltage");
            fc.max_power_watts   = g("servo.max_power_W");
            fc.cmd_timeout_s     = g("servo.default_timeout_s");
            fc.gear_reduction    = configs_[m].gear_reduction;
        } else {
            fc.max_acceleration = fc.max_velocity = fc.min_position =
            fc.max_position = fc.kp = fc.ki = fc.kd = fc.max_current_amps =
            fc.max_voltage_volts = fc.max_power_watts = fc.cmd_timeout_s = NAN;
            fc.gear_reduction = configs_[m].gear_reduction;
        }
    }
    moteus_ros_msg.can_device      = can_device_path_;
    moteus_ros_msg.motors_replying = static_cast<uint8_t>(motors_replying);
    feedback_pub_->publish(moteus_ros_msg);
    arm_feedback_pub->publish(arm_feedback_msg);

    // ── Stage 9: safety ─────────────────────────────────────────────────────
    checkFaults();
    checkAlerts();

    // ── Stage 10: at most one staggered flash-config read (conf get only) ───
    serviceConfigReads();
}

// ---------------------------------------------------------------------------
// Position limit alerts — warns when approaching configured bounds
// ---------------------------------------------------------------------------

void MoteusDriverNode::checkFaults() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        auto& t = telem_[i];
        if (!t.connected) continue;

        if (t.fault != last_fault_[i]) {
            if (t.fault != 0) {
                RCLCPP_ERROR(this->get_logger(),
                    "Motor %d (%s) FAULT! Code: %d — commands blocked until STOP sent",
                    i + 1, ARM_JOINTS[i].hardware_name, t.fault);
                publishLog("# FAULT motor " + std::to_string(i + 1)
                    + " (" + ARM_JOINTS[i].hardware_name
                    + ") code=" + std::to_string(t.fault));
            } else if (last_fault_[i] != 0) {
                RCLCPP_INFO(this->get_logger(),
                    "Motor %d (%s) fault cleared", i + 1, ARM_JOINTS[i].hardware_name);
                publishLog("# Motor " + std::to_string(i + 1)
                    + " (" + ARM_JOINTS[i].hardware_name + ") fault cleared");
            }
            last_fault_[i] = t.fault;
        }

        if (t.mode != last_mode_[i]) {
            if (t.mode == 1)
                RCLCPP_ERROR(this->get_logger(),
                    "Motor %d (%s) entered FAULT mode", i + 1, ARM_JOINTS[i].hardware_name);
            else if (t.mode == 0 && last_mode_[i] != 0)
                RCLCPP_WARN(this->get_logger(),
                    "Motor %d (%s) is STOPPED (driver disabled)", i + 1, ARM_JOINTS[i].hardware_name);
            last_mode_[i] = t.mode;
        }
    }
}

void MoteusDriverNode::checkAlerts() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        auto& t = telem_[i];
        if (!t.connected) continue;

        auto&  c       = configs_[i];
        float  padding = c.position_warn_rev_padding;
        bool near_max  = t.position >= (c.position_max - padding);
        bool near_min  = t.position <= (c.position_min + padding);

        if (near_max || near_min) {
            if (!position_alert_raised_[i]) {
                position_alert_raised_[i] = true;
                const char* which = near_max ? "MAX" : "MIN";
                float limit = near_max ? c.position_max : c.position_min;
                RCLCPP_WARN(this->get_logger(),
                    "Motor %d (%s) near %s position limit! pos=%.3f limit=%.3f (padding=%.3f)",
                    i + 1, ARM_JOINTS[i].hardware_name, which,
                    t.position, limit, padding);
                publishLog("# WARNING motor " + std::to_string(i + 1)
                    + " (" + ARM_JOINTS[i].hardware_name + ") near " + which
                    + " limit: pos=" + std::to_string(t.position)
                    + " limit=" + std::to_string(limit));
            }
        } else {
            position_alert_raised_[i] = false;
        }
    }
}

// ---------------------------------------------------------------------------
// Real-time config update — HMI publishes to /arm/config_update
//
// Applies a single "conf set <register> <value>" to the requested motor and
// updates the in-memory MotorConfig so subsequent feedback messages reflect
// the change.  The moteus firmware applies the value immediately (RAM only —
// not persisted to flash unless you separately run "conf write").
// ---------------------------------------------------------------------------

void MoteusDriverNode::configUpdateCallback(
        const rover_msgs::msg::MoteusConfigUpdate::SharedPtr msg)
{
    int idx = msg->motor_id;
    if (idx < 0 || idx >= NUM_MOTORS) {
        RCLCPP_WARN(this->get_logger(),
            "configUpdate: motor_id %d out of range (0..%d)", idx, NUM_MOTORS - 1);
        return;
    }
    if (msg->register_name.empty() || std::isnan(msg->value)) return;

    const std::string val_str = std::to_string(msg->value);
    const std::string cmd = "conf set " + msg->register_name + " " + val_str;

    RCLCPP_INFO(this->get_logger(),
        "configUpdate motor %d (%s): %s = %s",
        idx + 1, ARM_JOINTS[idx].hardware_name,
        msg->register_name.c_str(), val_str.c_str());
    publishLog("# conf motor " + std::to_string(idx + 1)
        + " (" + ARM_JOINTS[idx].hardware_name + "): "
        + msg->register_name + " = " + val_str);

    if (idx >= (int)controllers_.size()) return;   // transport absent

    controllers_[idx]->DiagnosticCommand(cmd);

    // For max_velocity, the moteus firmware has two registers that must both
    // be updated to cap velocity consistently during motion profile planning.
    if (msg->register_name == "servo.max_velocity") {
        controllers_[idx]->DiagnosticCommand(
            "conf set servo.default_velocity_limit " + val_str);
    } else if (msg->register_name == "servo.default_velocity_limit") {
        controllers_[idx]->DiagnosticCommand(
            "conf set servo.max_velocity " + val_str);
    }

    applyConfigToMemory(idx, msg->register_name, msg->value);
    readFlashConfig(idx);   // Actual column confirms from the controller itself
}

void MoteusDriverNode::applyConfigToMemory(int idx, const std::string& reg, float val) {
    auto& c = configs_[idx];
    if      (reg == "servo.pid_position.kp")        c.kp              = val;
    else if (reg == "servo.pid_position.ki")        c.ki              = val;
    else if (reg == "servo.pid_position.kd")        c.kd              = val;
    else if (reg == "servo.max_current_A")          c.max_current_A   = val;
    else if (reg == "servo.max_velocity"
          || reg == "servo.default_velocity_limit") c.max_velocity    = val;
    else if (reg == "servo.default_accel_limit")    c.max_acceleration = val;
    else if (reg == "servopos.position_min")        c.position_min    = val;
    else if (reg == "servopos.position_max")        c.position_max    = val;
    else if (reg == "servo.max_voltage")            c.max_voltage     = val;
    else if (reg == "servo.max_power_W")            c.max_power_W     = val;
    else if (reg == "servo.default_timeout_s")      c.def_timeout     = val;
}

// ---------------------------------------------------------------------------
// reInitTransport — re-create owned transport + controllers from fdcanusb detection.
// Uses TransportRegistry (not singleton) so the new transport is fully owned.
// ---------------------------------------------------------------------------
void MoteusDriverNode::reInitTransport() {
    controllers_.clear();
    transport_.reset();  // fd is now closed — refcount hit 0

    // Detect explicitly (rather than TransportRegistry::make({})) so we know
    // which device we opened and can notice when it re-enumerates.
    const std::string device = mot::Fdcanusb::DetectFdcanusb();
    if (device.empty()) {
        RCLCPP_ERROR(this->get_logger(),
            "No fdcanusb found (/dev/fdcanusb, /dev/serial/by-id/*fdcanusb*) — "
            "will keep retrying");
        can_device_path_.clear();
        return;
    }
    char resolved[PATH_MAX];
    can_device_path_ = ::realpath(device.c_str(), resolved) ? resolved : device;
    transport_ = std::make_shared<mot::Fdcanusb>(device);
    RCLCPP_INFO(this->get_logger(), "CAN transport open: %s (%s)",
                can_device_path_.c_str(), device.c_str());

    for (int id = 1; id <= NUM_MOTORS; id++) {
        mot::Controller::Options opts;
        opts.id        = id;
        opts.transport = transport_;
        opts.query_format.q_current = mot::kFloat;
        opts.query_format.power     = mot::kFloat;
        opts.query_format.aux2_gpio = mot::kInt8;
        opts.position_format.velocity_limit = mot::kFloat;   
        opts.position_format.accel_limit    = mot::kFloat;  
        controllers_.push_back(std::make_shared<mot::Controller>(opts));
    }
    // No blocking config sweep here — mark everything wanted and let
    // serviceConfigReads() repopulate one motor at a time. Displayed values
    // stay (they belong to the motors, not the link that just reopened).
    for (int m = 0; m < NUM_MOTORS; m++) cfg_read_wanted_[m] = true;
}

bool MoteusDriverNode::readFlashConfig(int m) {
    if (m >= (int)controllers_.size()) return false;
    auto ctrl = controllers_[m];
    // SetQuery gates on liveness: DiagnosticCommand to an absent motor hangs.
    if (!ctrl->SetQuery()) return false;
    ctrl->DiagnosticCommand("tel stop");  // kill tview stream leftovers
    ctrl->DiagnosticFlush();   // drop stale lines or every reply shifts by one
    std::map<std::string, float> out;
    for (const auto& [reg, val] : configs_[m].get_configs()) {
        // conf get answers a bare value line with no trailing OK
        const std::string raw = ctrl->DiagnosticCommand(
            "conf get " + reg, mot::Controller::kExpectSingleLine);
        // Empty = timed out (its late reply would shift every later
        // register); non-numeric/ERR = garbled. Abort — half-read sets
        // must never be published as valid.
        char* end = nullptr;
        const float v = std::strtof(raw.c_str(), &end);
        if (raw.empty() || end == raw.c_str() || raw.rfind("ERR", 0) == 0) {
            ctrl->DiagnosticFlush(1, 0.1);
            return false;
        }
        out[reg] = v;
    }
    if (out.empty()) return false;
    flash_cfg_[m] = std::move(out);
    flash_cfg_valid_[m] = true;
    return true;
}

// Sliced flash-config reader — called from run() after the telemetry decode.
// Does at most ONE bus transaction per poll tick (liveness+flush to start a
// motor, then a single conf get per tick), so telemetry never freezes: the
// old whole-motor read blocked the loop up to 4 s and the HMI kept painting
// stale values as live. conf get only; this path never writes to a motor.
void MoteusDriverNode::serviceConfigReads() {
    const int64_t now = steadyMs();

    if (cfg_read_motor_ < 0) {   // idle — pick the next motor that needs a read
        if (now < cfg_read_gate_ms_) return;
        for (int i = 0; i < NUM_MOTORS; i++) {
            const int m = (cfg_read_next_ + i) % NUM_MOTORS;
            if (!(cfg_read_wanted_[m] || !flash_cfg_valid_[m])) continue;
            if (!telem_[m].connected) continue;                  // never wait on ghosts
            if (now - cfg_read_last_ms_[m] < 5000) continue;     // failed read: retry later
            if (cfg_read_fails_[m] >= 3 && !cfg_read_wanted_[m]) continue;  // gave up; Refresh resets
            cfg_read_last_ms_[m] = now;
            auto ctrl  = controllers_[m];
            bool alive = false;
            try {
                alive = !!ctrl->SetQuery();
                if (alive) {
                    ctrl->DiagnosticCommand("tel stop");     // kill tview stream leftovers
                    ctrl->DiagnosticFlush(1, 0.05);          // drain stale lines
                }
            } catch (...) {}
            if (!alive) { cfg_read_gate_ms_ = steadyMs() + 250; return; }
            cfg_read_motor_ = m; cfg_read_reg_ = 0; cfg_read_accum_.clear();
            cfg_read_next_  = m + 1;
            return;
        }
        return;
    }

    // Mid-read: one register this tick.
    const int  m    = cfg_read_motor_;
    const auto regs = configs_[m].get_configs();
    if (cfg_read_reg_ < regs.size()) {
        auto ctrl = controllers_[m];
        const std::string reg = regs[cfg_read_reg_].first;
        std::pair<bool, float> out{false, NAN};
        try {
            const std::string raw = ctrl->DiagnosticCommand(
                "conf get " + reg, mot::Controller::kExpectSingleLine);
            // Validate hard (test-proven failure modes): a timed-out
            // command returns "" (strtof would fake 0.0), and its LATE
            // reply then shifts every later register by one. Reject
            // empty/non-numeric/ERR lines and drain the stragglers so
            // the retry starts clean.
            char* end = nullptr;
            const float v = std::strtof(raw.c_str(), &end);
            if (!raw.empty() && end != raw.c_str() && raw.rfind("ERR", 0) != 0)
                out = {true, v};
            else
                ctrl->DiagnosticFlush(1, 0.1);
        } catch (...) {}
        if (!out.first) {   // register read failed — abort this motor's pass
            cfg_read_motor_ = -1;
            cfg_read_gate_ms_ = steadyMs() + 250;
            if (++cfg_read_fails_[m] >= 3) {
                cfg_read_wanted_[m] = false;
                RCLCPP_WARN(this->get_logger(),
                    "M%d flash-config read failed 3x — giving up until next refresh", m + 1);
            }
            return;
        }
        cfg_read_accum_[reg] = out.second;
        cfg_read_reg_++;
        return;
    }

    // All registers in — commit atomically (never publish a half-read set).
    flash_cfg_[m]       = cfg_read_accum_;
    flash_cfg_valid_[m] = true;
    cfg_read_wanted_[m] = false;
    cfg_read_fails_[m]  = 0;
    cfg_read_motor_     = -1;
    cfg_read_gate_ms_   = now + 250;
}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoteusDriverNode>());
    rclcpp::shutdown();
    return 0;
}
