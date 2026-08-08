// death_ray_cbs_input_node.cpp  - For claude
/**
 * Panel -> death ray bridge. See header for the topic list and button layout.
 */
#include "death_ray_cbs_input_node.h"

// Constructor
DeathRayCbsInputNode::DeathRayCbsInputNode() : Node("death_raw_cbs_input_node")
{
    panel_sub_ = this->create_subscription<rover_msgs::msg::GenericPanel>(
        PANEL_TOPIC, rclcpp::QoS(ROS_QOS),
        std::bind(&DeathRayCbsInputNode::panelCallback, this, std::placeholders::_1));

    death_ray_position_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        DEATH_RAY_POSITION_TOPIC, rclcpp::QoS(ROS_QOS),
        std::bind(&DeathRayCbsInputNode::deathRayPositionCallback, this, std::placeholders::_1));

    death_ray_motor_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        DEATH_RAY_CMD_TOPIC, rclcpp::QoS(ROS_QOS));

    death_ray_zero_pub_ = this->create_publisher<std_msgs::msg::Empty>(
        DEATH_RAY_ZERO_TOPIC, rclcpp::QoS(ROS_QOS));

    RCLCPP_INFO(this->get_logger(), "DeathRayCbsInputNode initialization complete.");
}

// Main function (entry point of node)
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeathRayCbsInputNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// Member function implementations (sub callbacks go first)

/**
 * Callback for the panel subscriber.
 *
 * The panel publishes its entire state on every message, not just changes, so
 * we only act on rising edges (0 -> 1). Holding a button down does nothing
 * extra; each press is one jog.
 *
 * Home is checked first so that a mashed-buttons situation zeroes rather than moves.
 */
void DeathRayCbsInputNode::panelCallback(const rover_msgs::msg::GenericPanel::SharedPtr msg)
{
    // Panel might not be fully populated (or might be a different panel entirely)
    if ((int)msg->buttons.size() < NUM_USED_BUTTONS) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), WARN_THROTTLE_MS,
            "Panel message only has %zu buttons, need at least %d. Ignoring.",
            msg->buttons.size(), NUM_USED_BUTTONS);
        return;
    }

    int home = msg->buttons[BUTTON_INDEX__HOME];
    int ccw  = msg->buttons[BUTTON_INDEX__CCW];
    int cw   = msg->buttons[BUTTON_INDEX__CW];
    int tweak = msg->switches[SWITCH_INDEX__TWEAKER];
    int ctrl_mode = msg->switches[SWITCH_INDEX__MODE] ? PRECISION_INCREMENTAL : JOGGING_CONTINUOUS;

    if (isRisingEdge(BUTTON_INDEX__HOME, home)) {
        homeDeathRay();
    }
    else if (isRisingEdge(BUTTON_INDEX__CCW, ccw) && ctrl_mode == PRECISION_INCREMENTAL) {
        incDeathRay(-INC_STEP_DEG);
    }
    else if (isRisingEdge(BUTTON_INDEX__CW, cw) && ctrl_mode == PRECISION_INCREMENTAL) {
        incDeathRay(INC_STEP_DEG);
    } else if (ccw && ctrl_mode == JOGGING_CONTINUOUS) {
        if(tweak)
        {
            incDeathRay(-TWEAK_STEP_DEG);
        } else {
            incDeathRay(-JOG_STEP_DEG);

        }
    } else if (cw && ctrl_mode == JOGGING_CONTINUOUS) {
        if(tweak)
        {
            incDeathRay(TWEAK_STEP_DEG);
        } else {
            incDeathRay(JOG_STEP_DEG);

        }
    } 
    // Save state for the next message. Done after the checks, obviously.
    prev_buttons_[BUTTON_INDEX__HOME] = home;
    prev_buttons_[BUTTON_INDEX__CCW]  = ccw;
    prev_buttons_[BUTTON_INDEX__CW]   = cw;
}

/**
 * Callback for the death ray position feedback.
 *
 * The motor node is the source of truth for where the dish is, so we just cache
 * whatever it tells us and add our jog on top of it.
 *
 * Note: the motor node blocks while it steps, so this value is stale during a
 * move. That is fine here because we only send one command per button press.
 */
void DeathRayCbsInputNode::deathRayPositionCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    death_ray_position_deg_ = msg->data;
}

/**
 * Returns true only on a 0 -> 1 transition of the given button.
 */
bool DeathRayCbsInputNode::isRisingEdge(int button_index, int current_state)
{
    return (current_state == BUTTON_PRESSED) && (prev_buttons_[button_index] != BUTTON_PRESSED);
}

/**
 * Publish an absolute position command, current position + the requested jog.
 *
 * The motor node is in ABS mode, so commands are absolute targets, not deltas.
 * If DEATH_RAY_CONTROL_MODE is ever switched to REL, this becomes just
 * "publish degrees" and the position subscriber can be deleted.
 */
void DeathRayCbsInputNode::incDeathRay(float degrees)
{
    float target = death_ray_position_deg_ + degrees;

    void jogDeathRay(float degrees);
    // Clamp instead of dropping the command, so holding at the end of travel isn't confusing
    if (target > MAX_COMMAND_DEG) target = MAX_COMMAND_DEG;
    if (target < MIN_COMMAND_DEG) target = MIN_COMMAND_DEG;

    std_msgs::msg::Float32 msg;
    msg.data = target;
    death_ray_motor_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Jog %.1f deg -> commanding %.1f deg", degrees, target);
}

// void DeathRayCbsInputNode::jogDeathRay(float degrees)
// {
//     float target = death_ray_position_deg_ + degrees;

//     // Clamp instead of dropping the command, so holding at the end of travel isn't confusing
//     if (target > MAX_COMMAND_DEG) target = MAX_COMMAND_DEG;
//     if (target < MIN_COMMAND_DEG) target = MIN_COMMAND_DEG;

//     std_msgs::msg::Float32 msg;
//     msg.data = target;
//     death_ray_motor_pub_->publish(msg);

//     RCLCPP_INFO(this->get_logger(), "Jog %.1f deg -> commanding %.1f deg", degrees, target);
// }

/**
 * Tell the motor node that wherever the dish is right now is zero.
 *
 * This does not move the dish, it is a manual home: point it where you want
 * zero to be with the jog buttons, then hit the middle button.
 */
void DeathRayCbsInputNode::homeDeathRay()
{
    std_msgs::msg::Empty msg;
    death_ray_zero_pub_->publish(msg);

    // Keep our cached copy in sync so the next jog isn't off by the old position
    death_ray_position_deg_ = 0.0f;

    RCLCPP_INFO(this->get_logger(), "Death ray zeroed.");
}