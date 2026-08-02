// See lighting_translator.hpp for the topic layout.
#include "lighting_translator.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

// Constructor
LightingTranslatorNode::LightingTranslatorNode() : Node("lighting_translator")
{
    // Parameters (all of these live in config/lighting_translator.yaml)
    const std::string loop_name = this->declare_parameter<std::string>("loop_name", "on_board");
    const std::string lighting_topic_suffix =
        this->declare_parameter<std::string>("lighting_topic_suffix", "lighting");
    const std::string cmd_topic      = this->declare_parameter<std::string>("cmd_topic", "/lights/cmd");
    const std::string feedback_topic = this->declare_parameter<std::string>("feedback_topic", "/lights/feedback");
    const int qos_depth              = this->declare_parameter<int>("qos_depth", 15);

    // Our identity on the bus. comms keeps the device id in a file static, and
    // this node is its own process, so it has to be set here as well as in the
    // can node. Default is the invalid id, so an unconfigured node cannot
    // quietly send frames claiming to be someone else.
    const int device_id_param = this->declare_parameter<int>("device_id", COMMS_INVALID_DEVICE_ID);
    if (device_id_param < 0 || device_id_param > (int)COMMS_DEVICE_ID_MASK) {
        RCLCPP_FATAL(this->get_logger(),
                     "device_id %d is out of range, must be 0x0000 to 0x%04X (see device_ids.h)",
                     device_id_param, COMMS_DEVICE_ID_MASK);
        throw std::runtime_error("device_id parameter out of range");
    }
    comms_set_device_id((uint16_t)device_id_param);

    if (comms_get_device_id() == COMMS_INVALID_DEVICE_ID) {
        RCLCPP_WARN(this->get_logger(),
                    "device_id is unset. Feedback will still reach the HMI, but every "
                    "lighting command will be rejected. Set the device_id parameter.");
    }

    const std::string topic_base = "/can/" + loop_name + "/";

    // Nothing has been commanded or reported yet.
    last_commanded.fill(std::nan(""));
    last_feedback.fill(0.0);

    // Reliable, keep last N. Lighting traffic is sparse and a dropped command
    // leaves a light at the wrong brightness until someone touches it again.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable();

    can_outgoing_pub = this->create_publisher<GenericCommsMsg>(topic_base + "outgoing", qos);
    feedback_pub     = this->create_publisher<Float64MultiArray>(feedback_topic, qos);

    cmd_sub = this->create_subscription<Float64MultiArray>(
        cmd_topic, qos,
        std::bind(&LightingTranslatorNode::cmd_callback, this, std::placeholders::_1));

    can_lighting_sub = this->create_subscription<GenericCommsMsg>(
        topic_base + lighting_topic_suffix, qos,
        std::bind(&LightingTranslatorNode::can_lighting_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Lighting translator up as device 0x%04X",
                comms_get_device_id());
    RCLCPP_INFO(this->get_logger(), "  %s -> %soutgoing", cmd_topic.c_str(), topic_base.c_str());
    RCLCPP_INFO(this->get_logger(), "  %s%s -> %s",
                topic_base.c_str(), lighting_topic_suffix.c_str(), feedback_topic.c_str());
}


// Main function (entry point of node)
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // The constructor throws on a bad device id. Catch it so we exit with a
    // readable message instead of a raw terminate().
    try {
        auto node = std::make_shared<LightingTranslatorNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("lighting_translator"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}


// Member function implementations (sub and timer callbacks go first)

// The HMI publishes the whole desired array on every interaction, including the
// boards it did not touch. We only put a frame on the bus for the boards whose
// value actually moved - dragging a slider fires on every pixel of travel, and
// five frames per pixel is a lot of bus for no reason.
void LightingTranslatorNode::cmd_callback(const Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() < (size_t)NUM_LED_BOARDS) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), ERROR_LOG_PERIOD_MS,
                             "Ignoring lighting command with %zu values, expected %d",
                             msg->data.size(), NUM_LED_BOARDS);
        return;
    }

    for (uint8_t board = 0; board < NUM_LED_BOARDS; board++) {
        // A bad value from upstream must never reach the hardware.
        const double percent = std::clamp(msg->data[board], 0.0, 100.0);

        // The NaN in last_commanded makes this comparison false on the first
        // command, which is what we want - everything gets sent once.
        if (std::fabs(percent - last_commanded[board]) < VALUE_CHANGE_EPSILON_PERCENT) continue;

        send_led_command(board, percent);
    }
}

// One feedback frame arrives per board. The HMI wants all five at once, so we
// keep the last known value for each board and republish the whole array every
// time any board reports.
void LightingTranslatorNode::can_lighting_callback(const GenericCommsMsg::SharedPtr msg)
{
    // The lighting topic carries every type routed to it, which includes the
    // commands other nodes send. Only feedback frames are ours to translate.
    if (msg->type != MSG_TYPE__LED_PANEL_FEEDBACK) return;

    // decode() does not check that the payload is long enough for the type, so
    // check it here rather than letting unpack_payload read off the end.
    if (msg->payload.size() < LED_PANEL_PAYLOAD_BYTES ||
        msg->payload.size() > MAX_MESSAGE_PAYLOAD_SIZE_BYTES) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), ERROR_LOG_PERIOD_MS,
                             "Device 0x%04X sent a %zu byte lighting feedback, expected %zu",
                             msg->sender_id, msg->payload.size(), LED_PANEL_PAYLOAD_BYTES);
        return;
    }

    // Rebuild what the can node took apart, then let comms do the unpacking.
    encoded_msg_t encoded;
    std::memset(&encoded, 0, sizeof(encoded));
    encoded.id          = pack_id(msg->sender_id, msg->type);
    encoded.payload_len = (uint8_t)msg->payload.size();
    std::memcpy(encoded.payload, msg->payload.data(), msg->payload.size());

    msg_t decoded;
    std::memset(&decoded, 0, sizeof(decoded));
    if (!decode(&encoded, &decoded)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), ERROR_LOG_PERIOD_MS,
                             "Could not decode lighting feedback from device 0x%04X",
                             msg->sender_id);
        return;
    }

    const uint8_t board  = decoded.payload.led_panel_feedback.led_panel_index;
    const double percent = decoded.payload.led_panel_feedback.panel_percent;

    if (board >= NUM_LED_BOARDS) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), ERROR_LOG_PERIOD_MS,
                             "Device 0x%04X reported board index %u, only %d boards exist",
                             msg->sender_id, board, NUM_LED_BOARDS);
        return;
    }

    // Clamp here too. The HMI lights a glyph on this value, so a garbage percent
    // from a confused panel should not turn into a garbage display.
    last_feedback[board] = std::clamp(percent, 0.0, 100.0);

    Float64MultiArray out;
    out.data.assign(last_feedback.begin(), last_feedback.end());
    feedback_pub->publish(out);
}

// Builds one SET_LED_PWM frame and hands it to the can node. There is no
// destination address in this protocol - the panel picks its own frames out of
// the traffic by led_panel_index, so that index is the only thing aiming this.
void LightingTranslatorNode::send_led_command(uint8_t board_index, double percent)
{
    msg_t msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.type = MSG_TYPE__SET_LED_PWM;
    msg.payload.led_panel_cmd.led_panel_index = board_index;
    msg.payload.led_panel_cmd.panel_percent   = (float)percent;
    // is_response is deliberately left alone. It is not packed onto the wire,
    // and feedback is its own message type now.

    // sender_id is filled in by encode() from the device id parameter.
    encoded_msg_t encoded;
    std::memset(&encoded, 0, sizeof(encoded));
    if (!encode(&msg, &encoded)) {
        // encode() only fails here if the device id was never set.
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), ERROR_LOG_PERIOD_MS,
                              "Could not encode a lighting command for board %u. "
                              "Is the device_id parameter set?", board_index);
        return;
    }

    GenericCommsMsg out;
    out.sender_id   = id_device(encoded.id);
    out.type        = id_type(encoded.id);
    out.payload_len = encoded.payload_len;
    out.payload.assign(encoded.payload, encoded.payload + encoded.payload_len);

    can_outgoing_pub->publish(out);

    // Only record it once it is actually out the door, so a failed send gets
    // retried on the next command rather than being remembered as sent.
    last_commanded[board_index] = percent;
}