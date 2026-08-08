// Lighting translator.
//
// Sits between the HMI's lighting module and the generic CAN hardware interface.
// It does not touch CAN hardware - it only speaks ROS2 topics on both sides.
//
//   /lights/cmd            (Float64MultiArray, 5 x percent)  HMI  -> here
//   /can/<loop>/outgoing   (GenericCommsMsg)                 here -> can node -> bus
//   /can/<loop>/lighting   (GenericCommsMsg)                 bus  -> can node -> here
//   /lights/feedback       (Float64MultiArray, 5 x percent)  here -> HMI
//
// The HMI works in whole 5 board arrays, the bus works in one board per frame,
// so this node fans out commands and gathers feedback back up into an array.
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rover_msgs/msg/generic_comms_msg.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <comms.h>

#include <array>
#include <string>

// Board order is the HMI's board order (see lighting_module.h):
//   0 front left, 1 front right, 2 left, 3 right, 4 back
// led_panel_index on the wire uses the same numbering.
#define NUM_LED_BOARDS 5

// Payload sizes are written out by hand on purpose. sizeof(LedPanelCMD_t) is 12
// because of struct padding, the wire form is 5 bytes.
#define LED_PANEL_PAYLOAD_BYTES (sizeof(uint8_t) + sizeof(float))   // 5

// A board is only re commanded when its value moves by more than this. The HMI
// sliders are whole percent, so anything smaller is float noise or a feedback echo.
#define VALUE_CHANGE_EPSILON_PERCENT 0.05

// How long we wait between error logs, so one bad talker on the bus doesn't
// produce a million log lines a second.
#define ERROR_LOG_PERIOD_MS 1000

class LightingTranslatorNode : public rclcpp::Node {
public:
    LightingTranslatorNode();

private:
    using GenericCommsMsg   = rover_msgs::msg::GenericCommsMsg;
    using Float64MultiArray = std_msgs::msg::Float64MultiArray;

    // Data types and structs go first

    // Last value we actually put on the bus for each board. NaN until we have
    // sent anything, so the first command from the HMI goes out for all boards.
    std::array<double, NUM_LED_BOARDS> last_commanded;

    // Last value each board reported. Boards that have never reported stay at 0,
    // which the HMI draws as off - it only lights a glyph on confirmed state.
    std::array<double, NUM_LED_BOARDS> last_feedback;

    // SUBS AND PUBS
    rclcpp::Publisher<GenericCommsMsg>::SharedPtr can_outgoing_pub;
    rclcpp::Publisher<Float64MultiArray>::SharedPtr feedback_pub;

    rclcpp::Subscription<Float64MultiArray>::SharedPtr cmd_sub;
    void cmd_callback(const Float64MultiArray::SharedPtr msg);

    rclcpp::Subscription<GenericCommsMsg>::SharedPtr can_lighting_sub;
    void can_lighting_callback(const GenericCommsMsg::SharedPtr msg);

    // Member function prototypes
    void send_led_command(uint8_t board_index, double percent);
};