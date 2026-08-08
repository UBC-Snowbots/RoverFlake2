/**
 * death_ray_panel_control
 *
 * Dumb, event driven bridge between the control base station panel and the
 * death ray motor control node. Panel state in -> death ray commands out.
 * No motion logic lives here; that is all in death_ray_motor_control.
 *
 * Panel:  /cbs/left_panel_a  (GenericPanel)
 * Out:    death_ray/motor_commands (Float32), death_ray/zero (Empty)
 * In:     death_ray/position (Float32)  - so we know what to add our jog to
 */
#pragma once

#include "rclcpp/rclcpp.hpp"

//! Change this include (and the type below) to whatever package GenericPanel.msg actually lives in
#include "rover_msgs/msg/generic_panel.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"

#include <cstdint>

// Topics
#define PANEL_TOPIC              "/cbs/left_panel_a"
#define DEATH_RAY_CMD_TOPIC      "death_ray/motor_commands"
#define DEATH_RAY_ZERO_TOPIC     "death_ray/zero"
#define DEATH_RAY_POSITION_TOPIC "death_ray/position"

#define ROS_QOS 1

enum CTRL_MODE {
    UNKOWN,
    PRECISION_INCREMENTAL, // Very small, 0.1 degree changes
    JOGGING_CONTINUOUS // Continous, hold down movement.
};

/**
 * Panel button layout.
 *
 * The two buttons that showed up as 1 in the topic echo are indices 0 and 2,
 * so the button physically between them is index 1 -> used for homing.
 * If the panel is wired the other way around, swap CCW/CW here and nothing
 * else in this node needs to change.
 */
#define BUTTON_INDEX__CCW      0
#define BUTTON_INDEX__HOME     1
#define BUTTON_INDEX__CW       2
#define NUM_USED_BUTTONS       3

#define SWITCH_INDEX__MODE     3
#define SWITCH_INDEX__TWEAKER  4

#define BUTTON_PRESSED         1

// Degrees moved per button press. One press = one jog, holding does nothing extra.
#define INC_STEP_DEG           0.1f

#define JOG_STEP_DEG           2.0f
#define TWEAK_STEP_DEG           0.1f

/**
 * The motor node rejects anything outside (-180, 180], so clamp before we send.
 * -180 itself is rejected, hence the slightly-off minimum.
 */
#define MAX_COMMAND_DEG        180.0f
#define MIN_COMMAND_DEG       -179.0f

// How often we're allowed to complain about a malformed panel message (ms)
#define WARN_THROTTLE_MS       2000

/**
 * @brief Translates control panel button presses into death ray commands.
 */
class DeathRayCbsInputNode : public rclcpp::Node {
public:
    DeathRayCbsInputNode();

private:
    // Data types and structs go first
    float death_ray_position_deg_ = 0.0f;              // Last position reported by the motor node
    int prev_buttons_[NUM_USED_BUTTONS] = {0, 0, 0};   // Previous panel state, for rising edge detection

    // SUBS AND PUBS
    rclcpp::Subscription<rover_msgs::msg::GenericPanel>::SharedPtr panel_sub_;
    void panelCallback(const rover_msgs::msg::GenericPanel::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr death_ray_position_sub_;
    void deathRayPositionCallback(const std_msgs::msg::Float32::SharedPtr msg);

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr death_ray_motor_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr death_ray_zero_pub_;

    // Member function prototypes
    bool isRisingEdge(int button_index, int current_state);
    // void jogDeathRay(float degrees);
    void incDeathRay(float degrees);
    void homeDeathRay();
};