// mock_arm — a stand-in for moteus_driver that closes the /arm/command →
// /joint_states loop without hardware, so the HMI's send-command panel moves
// the RViz model.
//
// The simulated state is the motor COUNTER (output-shaft revolutions), exactly
// like the real driver: boot state is 0 on every axis, which after the
// 2026-08-09 calibration means "parked on the limit switches". Counters are
// converted to URDF angles through the same ARM_JOINTS map
// (motorRevToJointRad) the real driver will use, so RViz tracking correctly
// here verifies initial_pos_rad and the direction signs.
//
// Supported commands (same wire contract as the real driver, arm_commands.h):
//   CMD_ABS_VEL ('V')  velocities[] deg/s, NaN = skip, 0 = stop that axis
//   CMD_ABS_POS ('P')  positions[] output revs + velocities[] travel deg/s
//   CMD_STOP    ('S')  positions[] mask (non-NaN = stop that one), empty = all
//   CMD_HOME    ('H')  creep back toward counter 0 at homing speed
//   CMD_ZERO    ('Z')  re-zero the counter at the current position
//
// Limits mirror AxisConfig: velocity clamped to max_running_speed, position
// clamped to [min_position_rev, max_position_rev] (switch-referenced travel;
// min is 0 wherever the switch is a true end stop). The clamps emulate the
// hard stops: an axis driven into one stays there.
//
// Motor-space commands (cmd_value = CMD_SPACE_MOTOR) are treated as axis
// space — this model has no differential wrist; M5/M6 == A5/A6.

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/arm_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <rover_arm_common/arm_commands.h>
#include <rover_arm_common/motor_addressing.h>
#include <rover_arm_common/motor_config.h>

#include <algorithm>
#include <cmath>

namespace {
constexpr double TICK_S = 0.02;  // 50 Hz, same order as the real poll loop
}

class MockArmNode : public rclcpp::Node {
public:
    MockArmNode() : Node("mock_arm") {
        // Volatile KeepLast(1), matching the HMI publisher's QoS.
        auto qos = rclcpp::QoS(1).reliable().durability_volatile();
        cmd_sub_ = create_subscription<rover_msgs::msg::ArmCommand>(
            "/arm/command", qos,
            [this](rover_msgs::msg::ArmCommand::SharedPtr msg) { onCommand(msg); });

        js_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        timer_ = create_wall_timer(
            std::chrono::duration<double>(TICK_S), [this]() { tick(); });

        RCLCPP_INFO(get_logger(),
            "mock_arm: counters at 0 (homed pose). /arm/command -> /joint_states");
        // Calibration dump — rover_arm_common is header-only, so a stale
        // install/ can silently pin an old table into this binary. This makes
        // the running values visible instead of guessable.
        for (int a = 0; a < NUM_AXES; a++) {
            RCLCPP_INFO(get_logger(),
                "  %-3s %-16s switch=%+7.3f rad  dir=%+.0f  travel=[%+.2f, %+.2f] rev",
                ARM_JOINTS[a].hardware_name, ARM_JOINTS[a].urdf_joint_name,
                ARM_JOINTS[a].initial_pos_rad, ARM_JOINTS[a].direction,
                AxisConfig::min_position_rev[a], AxisConfig::max_position_rev[a]);
        }
    }

private:
    struct AxisSim {
        double pos_rev = 0.0;   // counter, 0 = limit switch
        double vel_revps = 0.0; // current commanded velocity
        double target_rev = NAN;  // NaN = velocity mode, else position target
    };

    void onCommand(const rover_msgs::msg::ArmCommand::SharedPtr& msg) {
        const char t = (char)msg->cmd_type;

        if (t == CMD_STOP) {
            const bool masked = !msg->positions.empty();
            for (int a = 0; a < NUM_AXES; a++) {
                if (masked && (a >= (int)msg->positions.size()
                               || std::isnan(msg->positions[a]))) continue;
                axes_[a].vel_revps = 0.0;
                axes_[a].target_rev = NAN;
            }
            return;
        }

        if (t == CMD_ABS_VEL) {
            for (int a = 0; a < NUM_AXES; a++) {
                double vd = (a < (int)msg->velocities.size()) ? msg->velocities[a] : NAN;
                if (std::isnan(vd)) continue;
                const double cap = AxisConfig::max_running_speed[a];
                axes_[a].vel_revps = std::clamp(degreesToRevolution(vd), -cap, cap);
                axes_[a].target_rev = NAN;
            }
            return;
        }

        if (t == CMD_ABS_POS) {
            for (int a = 0; a < NUM_AXES; a++) {
                double pos = (a < (int)msg->positions.size())  ? msg->positions[a]  : NAN;
                double vd  = (a < (int)msg->velocities.size()) ? msg->velocities[a] : NAN;
                if (std::isnan(pos) && std::isnan(vd)) continue;
                const double cap = AxisConfig::max_running_speed[a];
                if (std::isnan(pos)) {   // d pos nan v — pure velocity
                    axes_[a].vel_revps = std::clamp(degreesToRevolution(vd), -cap, cap);
                    axes_[a].target_rev = NAN;
                } else {                 // travel to pos at vd (or cap)
                    const double speed = std::isnan(vd)
                        ? cap : std::min(degreesToRevolution(std::fabs(vd)), (double)cap);
                    axes_[a].target_rev = pos;
                    axes_[a].vel_revps  = speed;
                }
            }
            return;
        }

        if (t == CMD_HOME) {
            auto homeOne = [this](int a) {
                if (a < 0 || a >= NUM_AXES) return;
                axes_[a].target_rev = 0.0;
                axes_[a].vel_revps  = AxisConfig::homing_speed_revps[a];
            };
            if (msg->cmd_value >= 0 && msg->cmd_value < NUM_AXES) homeOne(msg->cmd_value);
            else if (msg->cmd_value == HOME_VALUE_SELECTED)
                for (double v : msg->positions)
                    if (!std::isnan(v)) homeOne((int)v);
            else if (msg->cmd_value == HOME_VALUE_ALL_AXES_EXCEPT_EE)
                for (int a = 0; a < NUM_AXES; a++)
                    if (a != AXIS_EE_INDEX) homeOne(a);
            return;
        }

        if (t == CMD_ZERO) {
            for (int a = 0; a < NUM_AXES; a++) {
                double flag = (a < (int)msg->positions.size()) ? msg->positions[a] : NAN;
                if (std::isnan(flag)) continue;
                axes_[a].pos_rev = 0.0;   // "d exact 0" — reference moves, arm doesn't
                axes_[a].vel_revps = 0.0;
                axes_[a].target_rev = NAN;
            }
            return;
        }
    }

    void tick() {
        for (int a = 0; a < NUM_AXES; a++) {
            auto& ax = axes_[a];

            if (!std::isnan(ax.target_rev)) {
                // Position mode: step toward target, arrive exactly, stop.
                const double d = ax.target_rev - ax.pos_rev;
                const double step = std::fabs(ax.vel_revps) * TICK_S;
                if (std::fabs(d) <= step) {
                    ax.pos_rev = ax.target_rev;
                    ax.target_rev = NAN;
                    ax.vel_revps = 0.0;
                } else {
                    ax.pos_rev += std::copysign(step, d);
                }
            } else {
                ax.pos_rev += ax.vel_revps * TICK_S;
            }

            // Travel limits double as the hard stops. For most axes min is 0
            // (the limit switch IS the end of travel); A6's switch sits
            // mid-travel, so its range is symmetric around 0.
            const double lo = AxisConfig::min_position_rev[a];
            const double hi = AxisConfig::max_position_rev[a];
            if (ax.pos_rev <= lo)  { ax.pos_rev = lo;  if (ax.vel_revps < 0) ax.vel_revps = 0; }
            if (ax.pos_rev >= hi)  { ax.pos_rev = hi;  if (ax.vel_revps > 0) ax.vel_revps = 0; }
        }

        sensor_msgs::msg::JointState js;
        js.header.stamp = now();
        for (int a = 0; a < NUM_AXES; a++) {
            // joint_ee is fixed in dev_arm_description_v2 — publishing it
            // would only draw robot_state_publisher warnings.
            if (a == AXIS_EE_INDEX) continue;
            js.name.push_back(ARM_JOINTS[a].urdf_joint_name);
            js.position.push_back(motorRevToJointRad(a, axes_[a].pos_rev));
            js.velocity.push_back(motorRevPerSecToJointRadPerSec(a, axes_[a].vel_revps));
        }
        for (int g = 0; g < NUM_GRIPPER_JOINTS; g++) {
            js.name.push_back(GRIPPER_JOINT_NAMES[g]);
            js.position.push_back(0.0);
            js.velocity.push_back(0.0);
        }
        js_pub_->publish(js);
    }

    AxisSim axes_[NUM_AXES];
    rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockArmNode>());
    rclcpp::shutdown();
    return 0;
}
