// arm_twin — digital twin of the PHYSICAL arm.
//
// Where mock_arm integrates commands (what the arm was TOLD to do), this node
// renders telemetry (what the arm actually DID): it subscribes to
// /arm/feedback — the axis-space positions moteus_driver decodes from the CAN
// replies every poll cycle, wrist differential already inverted — and
// publishes /joint_states through the same ARM_JOINTS calibration. Point RViz
// at it and the model mirrors the real arm.
//
// Optionally it also measures twin-vs-sim error: set the sim_topic parameter
// (e.g. "/sim/joint_states", with mock_arm remapped there) and it publishes
// /arm/twin_error, the per-joint difference telemetry − sim in radians.
// That error is a HEALTH metric — calibration drift, missed steps, backlash,
// a stalled axis — not an IK input: MoveIt Servo computes IK from the twin's
// /joint_states directly.
//
// Typical wiring:
//   real arm     moteus_driver → /arm/feedback → arm_twin → /joint_states → RViz/Servo
//   sim overlay  mock_arm -r /joint_states:=/sim/joint_states
//                arm_twin -p sim_topic:=/sim/joint_states → /arm/twin_error

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/arm_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <rover_arm_common/motor_addressing.h>

#include <cmath>
#include <map>
#include <string>

class ArmTwinNode : public rclcpp::Node {
public:
    ArmTwinNode() : Node("arm_twin") {
        auto qos = rclcpp::QoS(1).reliable().durability_volatile();

        feedback_sub_ = create_subscription<rover_msgs::msg::ArmCommand>(
            "/arm/feedback", qos,
            [this](rover_msgs::msg::ArmCommand::SharedPtr msg) { onFeedback(msg); });

        js_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        const auto sim_topic = declare_parameter<std::string>("sim_topic", "");
        if (!sim_topic.empty()) {
            sim_sub_ = create_subscription<sensor_msgs::msg::JointState>(
                sim_topic, 10,
                [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); i++)
                        sim_positions_[msg->name[i]] = msg->position[i];
                });
            err_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
                "/arm/twin_error", 10);
            RCLCPP_INFO(get_logger(), "comparing against sim on %s -> /arm/twin_error",
                        sim_topic.c_str());
        }

        RCLCPP_INFO(get_logger(),
            "arm_twin: /arm/feedback (telemetry) -> /joint_states");
    }

private:
    void onFeedback(const rover_msgs::msg::ArmCommand::SharedPtr& msg) {
        sensor_msgs::msg::JointState js;
        js.header.stamp = now();

        std_msgs::msg::Float64MultiArray err;
        bool have_all_sim = err_pub_ != nullptr;

        for (int a = 0; a < NUM_AXES; a++) {
            if (a == AXIS_EE_INDEX) continue;   // joint_ee is fixed in the URDF
            const double rev = (a < (int)msg->positions.size()) ? msg->positions[a] : NAN;
            if (std::isnan(rev)) continue;      // motor not replying — skip, don't fake
            const char* name = ARM_JOINTS[a].urdf_joint_name;
            const double rad = motorRevToJointRad(a, rev);
            js.name.push_back(name);
            js.position.push_back(rad);

            if (err_pub_) {
                auto it = sim_positions_.find(name);
                if (it == sim_positions_.end()) { have_all_sim = false; }
                else err.data.push_back(rad - it->second);
            }
        }
        for (int g = 0; g < NUM_GRIPPER_JOINTS; g++) {
            js.name.push_back(GRIPPER_JOINT_NAMES[g]);
            js.position.push_back(0.0);
        }
        js_pub_->publish(js);

        if (err_pub_ && have_all_sim && !err.data.empty())
            err_pub_->publish(err);
    }

    rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr feedback_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sim_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr err_pub_;
    std::map<std::string, double> sim_positions_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmTwinNode>());
    rclcpp::shutdown();
    return 0;
}
