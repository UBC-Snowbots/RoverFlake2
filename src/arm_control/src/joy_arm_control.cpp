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
bool fk = false;
    // inputs[2] = msg->axes[0];
    // inputs[1] = msg->axes[1];
    // inputs[0] = msg->axes[5] - msg->axes[2];
    // inputs[3] = msg->axes[3];
    // inputs[4] = msg->axes[4];
    // inputs[5] = msg->axes[6];
    if(fk)
    {

    inputs[2] = msg->axes[0];
    inputs[1] = msg->axes[1];
    inputs[0] = msg->axes[5] - msg->axes[4];
    inputs[3] = msg->axes[3];
    inputs[4] = msg->axes[4];
    inputs[5] = msg->axes[6];

    if(CONTROL_MODE == POSITION_CONTROL){
        target.cmd_type = 'P';

    for (int i = 0; i < NUM_JOINTS; i++){
        target.positions[i] = axes[i].position + inputs[i] * 10;
    }
    }else if(CONTROL_MODE == VELOCITY_CONTROL){
        target.cmd_type = 'V';
    for (int i = 0; i < NUM_JOINTS; i++){
        target.velocities[i] = inputs[i] * 45;
    }
    }
    target.velocities[3] = 0;
    target.velocities[4] = 0;
    target.velocities[5] = 0;
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

    // ========== /arm/command for physical arm (velocity mode) ==========
    // TODO: Adapt this section when physical arm Cartesian control is needed.
    //       For now, only publish if there is actual button input.
    // rover_msgs::msg::ArmCommand target;
    // target.cmd_type = 'V';
    // target.velocities.resize(NUM_JOINTS, 0.0);
    // target.end_effector = 0.0;
    // Physical arm velocity commands would go here in the future.
    // arm_publisher->publish(target);
}


void ArmJoy::arm_callback(const rover_msgs::msg::ArmCommand::SharedPtr msg){

    for (int i = 0; i < NUM_JOINTS; i++){
        axes[i].position = msg->positions[i];
    }
}