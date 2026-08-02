// servo_to_arm_bridge.cpp — MoveIt Servo output -> /arm/command.
//
// Subscribes the JointTrajectory stream servo publishes (30 Hz, single-point
// trajectories) and converts each point to a CMD_ABS_POS ArmCommand:
// URDF radians -> output-shaft revs (inverse of motorRevToJointRad), velocity
// rad/s -> deg/s magnitude (wire contract). A1-A4 only; other joints ignored.
//
// SHADOW MODE (default: on) publishes to /arm/ik_command_shadow instead of
// /arm/command, so the whole IK stack can run against the live arm with zero
// motion while you sanity-check what WOULD be sent:
//   ros2 run arm_hardware_interface servo_to_arm_bridge                  # shadow
//   ros2 run arm_hardware_interface servo_to_arm_bridge --ros-args -p shadow:=false

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rover_msgs/msg/arm_command.hpp>
#include <rover_arm_common/motor_addressing.h>
#include <rover_arm_common/arm_commands.h>
#include <cmath>

class ServoToArmBridge : public rclcpp::Node {
public:
    ServoToArmBridge() : Node("servo_to_arm_bridge") {
        shadow_ = this->declare_parameter<bool>("shadow", true);
        const char* out = shadow_ ? "/arm/ik_command_shadow" : "/arm/command";
        pub_ = this->create_publisher<rover_msgs::msg::ArmCommand>(out, rclcpp::QoS(1));
        sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/arm_controller/joint_trajectory", rclcpp::QoS(10),
            std::bind(&ServoToArmBridge::onTrajectory, this, std::placeholders::_1));
        RCLCPP_WARN(this->get_logger(), "servo->arm bridge up, output: %s %s",
                    out, shadow_ ? "(SHADOW — no motion)" : "(LIVE)");
    }

private:
    void onTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        if (msg->points.empty()) return;
        const auto& pt = msg->points.front();

        rover_msgs::msg::ArmCommand cmd;
        cmd.cmd_type = CMD_ABS_POS;
        cmd.positions.resize(NUM_AXES, NAN);
        cmd.velocities.resize(NUM_AXES, NAN);

        for (size_t j = 0; j < msg->joint_names.size() && j < pt.positions.size(); j++) {
            int axis = -1;
            for (int a = AXIS_1_INDEX; a <= AXIS_4_INDEX; a++)
                if (msg->joint_names[j] == ARM_JOINTS[a].urdf_joint_name) { axis = a; break; }
            if (axis < 0) continue;

            const auto& jm = ARM_JOINTS[axis];
            // invert motorRevToJointRad: rad = init + dir * rev * 2pi
            cmd.positions[axis] = (pt.positions[j] - jm.initial_pos_rad) * jm.direction / (2.0 * M_PI);
            if (j < pt.velocities.size() && !std::isnan(pt.velocities[j]))
                cmd.velocities[axis] = std::fabs(pt.velocities[j]) * 180.0 / M_PI;  // wire = deg/s
        }
        pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "%s revs: A1=%.3f A2=%.3f A3=%.3f A4=%.3f",
            shadow_ ? "[shadow]" : "[LIVE]",
            cmd.positions[0], cmd.positions[1], cmd.positions[2], cmd.positions[3]);
    }

    bool shadow_;
    rclcpp::Publisher<rover_msgs::msg::ArmCommand>::SharedPtr pub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoToArmBridge>());
    rclcpp::shutdown();
    return 0;
}
