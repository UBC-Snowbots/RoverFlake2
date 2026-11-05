#include "arm_driver_node.h"
#include "axis_5_6_differential.h"
#include <cmath>
#include <cstdlib>
#include <thread>
#include <cstdio>      // popen / pclose
#include <sys/wait.h>  // WIFEXITED / WEXITSTATUS
#include <stdexcept>

static constexpr uint32_t ARM_RUN_RATE_MS = (1000u / 100u); // 100 hz
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
void combine_wrist(const MotorCommand& a5, const MotorCommand& a6,
                   MotorCommand& m5, MotorCommand& m6) {
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
    } else {
        differential_drive((float)a5.velocity, (float)a6.velocity, o5, o6);
        m5.velocity = o5;  m6.velocity = o6;
        m5.position = m6.position = NAN;
    }
}

std::vector<std::string> get_arm_motor_names() {
    std::vector<std::string> names;
    for (int i = 0; i < NUM_MOTORS; i++) {
        names.push_back(ARM_JOINTS[i].hardware_name);
    }
    return names;
}
} // namespace

// =============================================================================
// ArmDriverNode implementation
//
// For protocol details, units, and data field contents see moteus_protocol.h.
// For joint mapping and direction signs see motor_addressing.h.
// For per-motor PID/limits see motor_config.h.
// =============================================================================

ArmDriverNode::ArmDriverNode() : MoteusNode("moteus_driver", "arm", get_arm_configuration(), get_arm_motor_names()) {

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

    auto qos     = rclcpp::QoS(5).reliable().durability_volatile();

    feedback_pub_  = this->create_publisher<rover_msgs::msg::MoteusArmStatus>("/arm/moteus_feedback", qos);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);
    arm_feedback_pub = this->create_publisher<rover_msgs::msg::ArmCommand>("/arm/feedback", qos);

    command_sub_ = this->create_subscription<rover_msgs::msg::ArmCommand>(
        "/arm/command", qos,
        std::bind(&ArmDriverNode::commandCallback, this, std::placeholders::_1));

        
        // TODO Calibration in a future PR

    // calib_pub_ = this->create_publisher<rover_msgs::msg::MoteusCalibrationStatus>(
    //     "/arm/calibration_status", qos);
    // calib_sub_ = this->create_subscription<rover_msgs::msg::MoteusCalibrationRequest>(
    //     "/arm/calibration_request", qos,
    //     std::bind(&ArmDriverNode::calibrationCallback, this, std::placeholders::_1));

        #ifndef SKIP_CALIBRATION
        // configureMotors();
        #else 
        RCLCPP_WARN(this->get_logger(), "SKIPPING MOTOR CALIBRATIONS");
        #endif

    

    RCLCPP_INFO(this->get_logger(),
        "Moteus driver started: polling %d motors at 100 Hz", NUM_MOTORS);
}

// ---------------------------------------------------------------------------
// Command callback — ROS topic → pending_cmds_[]
// (See arm_commands.h for command code definitions and MotorCommand struct.)
// ---------------------------------------------------------------------------

void ArmDriverNode::commandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    char cmd_type = msg->cmd_type;

    if (cmd_type == CMD_STOP) {
        for (int a = 0; a < NUM_AXES; a++) { pending_axis_cmds_[a].active = true;
                                             pending_axis_cmds_[a].is_stop = true; }
        RCLCPP_INFO(this->get_logger(), "Command: STOP ALL");
        return;
    }

    if (cmd_type == CMD_ABS_POS) {
        for (int a = 0; a < NUM_AXES; a++) {
            double pos = (a < (int)msg->positions.size())  ? msg->positions[a]  : NAN;
            double vd  = (a < (int)msg->velocities.size()) ? msg->velocities[a] : NAN;
            if (std::isnan(pos) && std::isnan(vd)) continue;
            auto& p = pending_axis_cmds_[a];
            p.active = true; p.is_stop = false; p.is_zero = false;
            p.position = pos; p.velocity = degreesToRevolution(vd); p.max_torque = NAN;
        }
        return;
    }

    if (cmd_type == CMD_ABS_VEL) {
        for (int a = 0; a < NUM_AXES; a++) {
            
            double vd = (a < (int)msg->velocities.size()) ? msg->velocities[a] : NAN;
            if (std::isnan(vd)) continue; // Was breaking?
            auto& p = pending_axis_cmds_[a];
            p.active = true; p.is_stop = false; p.is_zero = false;
            p.position = NAN;
            p.velocity = degreesToRevolution(vd);
            // p.max_velocity = degreesToRevolution(vd);
            p.max_velocity = AxisConfig::max_running_speed[a];
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
        else if (msg->cmd_value == HOME_VALUE_ALL_AXES_EXCEPT_EE) { home_axis(AXIS_5_INDEX); return; }
        else if (msg->cmd_value == HOME_VALUE_ALL_AXES_AND_EE)    { /* EE TODO */ return; }
        RCLCPP_WARN(this->get_logger(), "Unknown home command value: %i", msg->cmd_value);
        return;
    }
    RCLCPP_WARN(this->get_logger(), "Unknown cmd_type: '%c' (%d)", cmd_type, (int)cmd_type);
}


void ArmDriverNode::home_axis(uint8_t index)
{
    // Just set axis state to request homing
    this->axes[index].state = AxisState::REQUESTING_HOMING;
}

void ArmDriverNode::prepareCommands() {
    // ── Stage 0: fresh axis_cmds_ every cycle (no stale carry) ──────────────
    for (auto& c : axis_cmds_) c.active = false;

    // ── Stage 1: intake — ROS -> axis_cmds_ (only for axes the SM doesn't own)
    bool stop_all = false;
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        for (int a = 0; a < NUM_AXES; a++) {
            auto& p = pending_axis_cmds_[a];
            if (!p.active) continue;
            if (p.is_stop)                              stop_all = true;
            else if (!stateMachineOwns(axes[a].state))  axis_cmds_[a] = p;
        }
        pending_axis_cmds_ = {};
    }
    if (stop_all) {
        for (int a = 0; a < NUM_AXES; a++) {
            axis_cmds_[a].active = true; axis_cmds_[a].is_stop = true;
            if (stateMachineOwns(axes[a].state)) axes[a].state = AxisState::RUNNING_OK; // abort homing
        }
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

    // ── Stage 3: wrist coherence — make axes 4&5 same mode + shared limits ──
    // Physically the two motors always move as a pair, so one shared vel/accel.
    {
        auto& a5 = axis_cmds_[AXIS_5_INDEX];
        auto& a6 = axis_cmds_[AXIS_6_INDEX];
        if (a5.active || a6.active) {
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
    {

        if (m == AXIS_5_INDEX || m == AXIS_6_INDEX) continue;
        motor_cmds_[m] = axis_cmds_[m];   // straight-through
         
        combine_wrist(axis_cmds_[AXIS_5_INDEX], axis_cmds_[AXIS_6_INDEX], motor_cmds_[AXIS_5_INDEX], motor_cmds_[AXIS_6_INDEX]);
    }

    RCLCPP_DEBUG(this->get_logger(), "Axis 5/6 Converted from Axis Space to Motor Space \n VELOCITY: %0.3f %0.3f -> %0.3f %0.3f \n", axis_cmds_[AXIS_5_INDEX].velocity, axis_cmds_[AXIS_6_INDEX].velocity, motor_cmds_[AXIS_5_INDEX].velocity, motor_cmds_[AXIS_6_INDEX].velocity);
    RCLCPP_DEBUG(this->get_logger(), "POSITION: %0.3f %0.1f -> %0.3f %0.3f \n", axis_cmds_[AXIS_5_INDEX].position, axis_cmds_[AXIS_6_INDEX].position, motor_cmds_[AXIS_5_INDEX].position, motor_cmds_[AXIS_6_INDEX].position);
    
    // ── Stage 5: motor-space zero side-channel (diagnostic, no frame) ───────
    for (int m = 0; m < NUM_MOTORS; m++) {
        if (motor_zero_req_[m]) { zero_position(m); motor_zero_req_[m] = false;
                                  motor_cmds_[m].active = false; }
    }
}

void ArmDriverNode::handleTelemetry() {
    rover_msgs::msg::MoteusArmStatus moteus_ros_msg;
    rover_msgs::msg::ArmCommand arm_feedback_msg;
    arm_feedback_msg.positions.resize(NUM_AXES);
    moteus_ros_msg.status.resize(NUM_AXES);
    moteus_ros_msg.config.resize(NUM_AXES);
    moteus_ros_msg.limit_switches.resize(NUM_AXES);

    // ── Stage 8: decode -> telem_, motor->axis (inverse for wrist) ──────────
    for (int id = 1; id <= NUM_MOTORS; id++) {
        auto& t = telem_[id - 1];
        if (!t.connected) continue;

        axes[id - 1].limit_switch = t.limit_switch;
        moteus_ros_msg.status[id -1].curr_current_amps = t.q_current;
        moteus_ros_msg.status[id -1].curr_torque = t.torque;
        moteus_ros_msg.status[id -1].curr_position = t.position;
        moteus_ros_msg.status[id -1].curr_power_watts = t.power;
        moteus_ros_msg.status[id -1].curr_velocity = t.velocity;
        moteus_ros_msg.status[id -1].moteus_fault = t.fault;

        moteus_ros_msg.status[id -1].driver_temp_degreesc = t.temperature;
        moteus_ros_msg.limit_switches[id - 1] = t.limit_switch;
        if (id - 1 != AXIS_5_INDEX && id - 1 != AXIS_6_INDEX) {
            axes[id - 1].position = t.position;
        } else if (id - 1 == AXIS_6_INDEX) {  // both wrist replies in — invert together
            float a5p, a6p;
            differential_drive_inverse(telem_[AXIS_5_INDEX].position, telem_[AXIS_6_INDEX].position, a5p, a6p);
            axes[AXIS_5_INDEX].position = a5p; axes[AXIS_6_INDEX].position = a6p;
        }
        arm_feedback_msg.positions[id - 1] = axes[id -1].position;
    }
    feedback_pub_->publish(moteus_ros_msg);
    arm_feedback_pub->publish(arm_feedback_msg);
}


// ---------------------------------------------------------------------------
// Calibration support
//
// Flow per motor:
//   1. Set calibrating_ = true  → run() returns early, CAN bus goes idle
//   2. Wait 50 ms for any in-flight BlockingCycle to finish
//   3. Release transport_ and controllers_ so the CAN socket is closed
//   4. Run: python3 -m moteus.moteus_tool -t <id>
//              --calibrate --cal-motor-poles 16 --cal-force-kv 265 --cal-hal
//   5. Re-create transport_ + controllers_
//   6. conf set motor_position.sources.0.type type.hall:4
//   7. conf save
//   8. Re-configure all motors (re-push PID/limits)
//   9. Set calibrating_ = false
// ---------------------------------------------------------------------------

void ArmDriverNode::publishCalibStatus(int motor_id, int state, const std::string& message) {
    //! Calibration is not implementet yet
    // rover_msgs::msg::MoteusCalibrationStatus s;
    // s.motor_id = motor_id;
    // s.state    = state;
    // s.message  = message;
    // calib_pub_->publish(s);
    // publishLog("# calib motor " + std::to_string(motor_id) + ": " + message);
    // RCLCPP_INFO(this->get_logger(), "[calib] motor %d: %s", motor_id, message.c_str());
}


//TODO Calibration in follow up PR (maybe after comp)
// ---------------------------------------------------------------------------
// runCalibration — full Hall-sensor calibration workflow for one motor.
//
// Step 1  Pause run loop (calibrating_ = true).
// Step 2  conf write — flush current RAM config to flash so our PID/limits
//         survive the calibration (moteus_tool may issue its own conf write).
// Step 3  Release transport — drops the only shared_ptr so the CAN fd closes.
//         This is safe because we used TransportRegistry::make() (not the
//         singleton), so we truly own the fd.
// Step 4  Run moteus_tool via popen with -u (unbuffered) so every output line
//         is streamed to /arm/config_log in real time.
// Step 5  Re-init transport + controllers.
// Step 6  conf set motor_position.sources.0.type type.hall:4
//         conf write — persist Hall sensor type to flash.
// Step 7  configureMotors() — re-push PID/limits into RAM.
// Step 8  Resume run loop.
// ---------------------------------------------------------------------------
void ArmDriverNode::runCalibration(int motor_can_id) {
    RCLCPP_ERROR(this->get_logger(), "Sorry, calibration is under construction... coming soon to a PR near you");
    // using namespace std::chrono_literals;

    // // ── Step 1: pause poll ────────────────────────────────────────────────────
    // calibrating_.store(true);
    // std::this_thread::sleep_for(60ms);  // let any in-flight BlockingCycle finish

    // // ── Step 2: conf write — persist current config before calibration ────────
    // // publishCalibStatus(motor_can_id, 1, "Saving config to flash before calibration...");
    // // publishLog("# calib motor " + std::to_string(motor_can_id) + ": conf write (pre-calib)");
    // // controllers_[motor_can_id - 1]->DiagnosticCommand("conf write");

    // // ── Step 3: release CAN fd ────────────────────────────────────────────────
    // publishCalibStatus(motor_can_id, 1, "Releasing CAN bus for moteus_tool...");
    // controllers_.clear();
    // transport_.reset();  // refcount → 0: CAN fd is now closed
    // std::this_thread::sleep_for(100ms);  // small grace period for OS to release the port

    // // ── Step 4: run moteus_tool, stream output line-by-line to command log ────
    // std::string cmd =
    //     "python3 -u -m moteus.moteus_tool"
    //     " -t " + std::to_string(motor_can_id) +
    //     " --calibrate"
    //     " --cal-motor-poles 16"
    //     " --cal-force-kv 265"
    //     " --cal-hal"
    //     " 2>&1";  // merge stderr so we see errors too

    // publishCalibStatus(motor_can_id, 1,
    //     "Running: " + cmd.substr(0, cmd.size() - 5));  // hide 2>&1 from HMI
    // publishLog("# calib motor " + std::to_string(motor_can_id) + ": " + cmd);

    // FILE* pipe = popen(cmd.c_str(), "r");
    // int exit_code = 1;
    // if (!pipe) {
    //     publishCalibStatus(motor_can_id, 3, "popen failed — cannot launch moteus_tool");
    // } else {
    //     char buf[512];
    //     while (fgets(buf, sizeof(buf), pipe)) {
    //         std::string line(buf);
    //         if (!line.empty() && line.back() == '\n') line.pop_back();
    //         publishLog("  [moteus_tool] " + line);
    //     }
    //     exit_code = pclose(pipe);
    //     // pclose returns the wait-status; extract real exit code
    //     if (WIFEXITED(exit_code)) exit_code = WEXITSTATUS(exit_code);
    // }

    // // ── Step 5: re-init transport ─────────────────────────────────────────────
    // reInitTransport();

    // if (exit_code != 0) {
    //     publishCalibStatus(motor_can_id, 3,
    //         "moteus_tool exited with code " + std::to_string(exit_code) + " — calibration FAILED");
    //     configureMotors();
    //     calibrating_.store(false);
    //     return;
    // }

    // // ── Step 6: set Hall sensor source and write to flash ─────────────────────
    // publishCalibStatus(motor_can_id, 1,
    //     "Setting motor_position.sources.0.type = type.hall:4 ...");
    // publishLog("# calib motor " + std::to_string(motor_can_id)
    //     + ": conf set motor_position.sources.0.type type.hall:4");
    // controllers_[motor_can_id - 1]->DiagnosticCommand(
    //     "conf set motor_position.sources.0.type type.hall:4");

    // // publishLog("# calib motor " + std::to_string(motor_can_id) + ": conf write (post-calib)");
    // // controllers_[motor_can_id - 1]->DiagnosticCommand("conf write");

    // // ── Step 7: re-push PID/limits into RAM ───────────────────────────────────
    // configureMotors();

    // publishCalibStatus(motor_can_id, 2,
    //     "Calibration complete.  Hall sensor type set and saved to flash.");

    // // ── Step 8: resume poll ───────────────────────────────────────────────────
    // calibrating_.store(false);
}

//TODO Calibration in future PR
void ArmDriverNode::calibrationCallback(
        const rover_msgs::msg::MoteusCalibrationRequest::SharedPtr msg)
{
    RCLCPP_ERROR(this->get_logger(), "Calibration is not implemented\n");
    // if (!calib_mutex_.try_lock()) {
    //     RCLCPP_WARN(this->get_logger(),
    //         "Calibration already in progress — request ignored.");
    //     return;
    // }
    // // mutex held; release it inside the thread after calibration finishes
    // std::thread([this, motor_id = msg->motor_id]() {
    //     if (motor_id == 0) {
    //         // Sequential: calibrate all motors one by one
    //         for (int id = 1; id <= NUM_MOTORS; id++) {
    //             runCalibration(id);
    //             std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //         }
    //     } else if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
    //         runCalibration(motor_id);
    //     } else {
    //         RCLCPP_WARN(this->get_logger(),
    //             "calibrationCallback: motor_id %d out of range", motor_id);
    //     }
    //     calib_mutex_.unlock();
    // }).detach();
}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmDriverNode>());
    rclcpp::shutdown();
    return 0;
}
