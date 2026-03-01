#include "joy_arm_control.h"
#include <cmath>

// Constructor
ArmJoy::ArmJoy() : 
Node("arm_joy_control") 
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    arm_publisher = this->create_publisher<rover_msgs::msg::ArmCommand>("/arm/command", qos);

    joy_vibrator = this->create_publisher<sensor_msgs::msg::JoyFeedback>("/joy/feedback", qos);

    twist_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_node/delta_twist_cmds", 10);

    joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&ArmJoy::joy_callback, this, std::placeholders::_1));
    arm_subscriber = this->create_subscription<rover_msgs::msg::ArmCommand>(
        "/arm/feedback", 10, std::bind(&ArmJoy::arm_callback, this, std::placeholders::_1));

    // ===== Servo → Physical Arm Bridge =====
    // Subscribe to MoveIt Servo's JointTrajectory output.  The callback
    // converts IK-solved joint velocities (rad/s, MoveIt frame) into
    // ArmCommand velocities (deg/s, firmware frame) and publishes on /arm/command.
    traj_subscriber = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/arm_controller/joint_trajectory", 10,
        std::bind(&ArmJoy::trajectory_callback, this, std::placeholders::_1));

    // Map URDF joint names → firmware axis indices (matches SRDF 'arm' group)
    urdf_to_axis_ = {
        {"shoulder_joint", 0},
        {"link_1_joint",   1},
        {"link1_link2",    2},
        {"a4_rotation",    3},
        {"a5_rotation",    4},
        {"a6_rotation",    5},
    };

    RCLCPP_INFO(this->get_logger(), "=== ArmJoy started ===");
#if ACTIVE_CONTROLLER == CONTROLLER_PRO_CONTROLLER
    RCLCPP_INFO(this->get_logger(), "Active controller: Nintendo Switch Pro Controller");
#elif ACTIVE_CONTROLLER == CONTROLLER_CYBORG_STICK
    RCLCPP_INFO(this->get_logger(), "Active controller: Saitek Cyborg USB Stick");
#else
    RCLCPP_INFO(this->get_logger(), "Active controller: Unknown");
#endif
    RCLCPP_INFO(this->get_logger(), "Cartesian frame: %s  |  Button speed: %.2f",
        ControllerConfig::CART_FRAME_ID, ControllerConfig::CART_BUTTON_SPEED);
}

// Program Entry Point
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmJoy>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}

// ---------- Helpers ----------

bool ArmJoy::btnPressed(const sensor_msgs::msg::Joy::SharedPtr& msg, int idx) {
    return idx >= 0 && idx < static_cast<int>(msg->buttons.size()) && msg->buttons[idx];
}

// ---------- Callbacks ----------

void ArmJoy::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
    if(this->fk)
    {
    rover_msgs::msg::ArmCommand target;
    target.positions.resize(NUM_JOINTS);
    target.velocities.resize(NUM_JOINTS);
    float target_positions[6];
    float inputs[6];
    inputs[2] = msg->axes[0];
    inputs[1] = msg->axes[1];
    inputs[0] = msg->axes[5] - msg->axes[4];
    inputs[3] = msg->axes[3];
    inputs[4] = (msg->buttons[0] - msg->buttons[3]);
    inputs[5] = (msg->buttons[1] - msg->buttons[2]);

    target.end_effector = (msg->buttons[9] -  msg->buttons[10]) * 60;

    if(CONTROL_MODE == POSITION_CONTROL){
        target.cmd_type = 'P';

    for (int i = 0; i < NUM_JOINTS; i++){
        target.positions[i] = axes[i].position + inputs[i] * 10;
    }
    }else if(CONTROL_MODE == VELOCITY_CONTROL){
        target.cmd_type = 'V';
    for (int i = 0; i < NUM_JOINTS; i++){
        target.velocities[i] = inputs[i] * 10;
    }
    }
    // target.velocities[3] = 0;
    // target.velocities[4] = 0;
    // target.velocities[5] = 0;
        arm_publisher->publish(target);
    } else {


    // --- Also publish TwistStamped for MoveIt Servo (drives RViz arm) ---
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    twist_msg->header.stamp = this->get_clock()->now();
    twist_msg->header.frame_id = ControllerConfig::CART_FRAME_ID;

    const double speed = ControllerConfig::CART_BUTTON_SPEED;

    // Start at zero — only add from pressed buttons
    double lx = 0.0, ly = 0.0, lz = 0.0;

    if (btnPressed(msg, ControllerConfig::BTN_CART_POS_X)) lx += speed;
    if (btnPressed(msg, ControllerConfig::BTN_CART_NEG_X)) lx -= speed;
    if (btnPressed(msg, ControllerConfig::BTN_CART_POS_Y)) ly += speed;
    if (btnPressed(msg, ControllerConfig::BTN_CART_NEG_Y)) ly -= speed;
    if (btnPressed(msg, ControllerConfig::BTN_CART_POS_Z)) lz += speed;
    if (btnPressed(msg, ControllerConfig::BTN_CART_NEG_Z)) lz -= speed;

    twist_msg->twist.linear.x  = lx;
    twist_msg->twist.linear.y  = ly;
    twist_msg->twist.linear.z  = lz;

    // --- EE Orientation from analog sticks ---
    auto applyDeadzone = [](double val, double dz) -> double {
        return (std::abs(val) < dz) ? 0.0 : val;
    };

    double ax_roll  = 0.0, ax_pitch = 0.0, ax_yaw = 0.0;
    const double rot_speed = ControllerConfig::ROT_STICK_SPEED;
    const double dz = ControllerConfig::AXIS_DEADZONE;

    if (ControllerConfig::AXIS_ROLL >= 0 &&
        ControllerConfig::AXIS_ROLL < static_cast<int>(msg->axes.size())) {
        ax_roll = applyDeadzone(msg->axes[ControllerConfig::AXIS_ROLL], dz) * rot_speed;
        if (ControllerConfig::INVERT_ROLL) ax_roll = -ax_roll;
    }
    if (ControllerConfig::AXIS_PITCH >= 0 &&
        ControllerConfig::AXIS_PITCH < static_cast<int>(msg->axes.size())) {
        ax_pitch = applyDeadzone(msg->axes[ControllerConfig::AXIS_PITCH], dz) * rot_speed;
        if (ControllerConfig::INVERT_PITCH) ax_pitch = -ax_pitch;
    }
    if (ControllerConfig::AXIS_YAW >= 0 &&
        ControllerConfig::AXIS_YAW < static_cast<int>(msg->axes.size())) {
        ax_yaw = applyDeadzone(msg->axes[ControllerConfig::AXIS_YAW], dz) * rot_speed;
        if (ControllerConfig::INVERT_YAW) ax_yaw = -ax_yaw;
    }

    twist_msg->twist.angular.x = ax_roll;
    twist_msg->twist.angular.y = ax_pitch;
    twist_msg->twist.angular.z = ax_yaw;

    bool is_moving = (lx != 0.0 || ly != 0.0 || lz != 0.0);
    bool is_rotating = (ax_roll != 0.0 || ax_pitch != 0.0 || ax_yaw != 0.0);
    if (is_moving || is_rotating) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Twist: lin[%.2f, %.2f, %.2f] ang[%.2f, %.2f, %.2f] frame=%s",
            lx, ly, lz, ax_roll, ax_pitch, ax_yaw,
            ControllerConfig::CART_FRAME_ID);
    }

    twist_publisher->publish(std::move(twist_msg));
    }

    // ========== Gripper toggle (edge-triggered) ==========
    bool gripper_btn = btnPressed(msg, ControllerConfig::BTN_GRIPPER_TOGGLE);
    if (gripper_btn && !prev_gripper_btn_) {
        gripper_open_ = !gripper_open_;
        RCLCPP_INFO(this->get_logger(), "Gripper %s",
            gripper_open_ ? "OPEN" : "CLOSED");
    }
    prev_gripper_btn_ = gripper_btn;

    // NOTE: ArmCommand (joint velocities + gripper) is now published in
    // trajectory_callback(), which fires whenever MoveIt Servo outputs a
    // JointTrajectory.  This ensures the physical arm receives the actual
    // IK-solved velocities rather than zeros.
}


void ArmJoy::trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    if (msg->points.empty()) return;

    auto cmd = std::make_unique<rover_msgs::msg::ArmCommand>();
    cmd->cmd_type = 'V';
    cmd->velocities.resize(NUM_JOINTS, 0.0);
    cmd->positions.resize(NUM_JOINTS, 0.0);
    cmd->end_effector = gripper_open_ ? 1.0 : 0.0;

    const auto& point = msg->points.back();

    for (size_t j = 0; j < msg->joint_names.size(); ++j) {
        auto it = urdf_to_axis_.find(msg->joint_names[j]);
        if (it == urdf_to_axis_.end()) continue;
        int axis = it->second;

        // Convert rad/s → deg/s and apply firmware direction
        if (axis >= 0 && axis < NUM_JOINTS) {
            double vel_rad = (j < point.velocities.size()) ? point.velocities[j] : 0.0;
            cmd->velocities[axis] = vel_rad * (180.0 / M_PI) * AXIS_DIR[axis];

            if (j < point.positions.size()) {
                cmd->positions[axis] = point.positions[j];
            }
        }
    }

    arm_publisher->publish(std::move(cmd));
}

void ArmJoy::arm_callback(const rover_msgs::msg::ArmCommand::SharedPtr msg){

    for (int i = 0; i < NUM_JOINTS; i++){
        axes[i].position = msg->positions[i];
    }
}