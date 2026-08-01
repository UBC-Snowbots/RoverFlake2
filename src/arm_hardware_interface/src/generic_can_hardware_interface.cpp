// Generic CAN hardware interface. See can_node.h for the topic layout.
#include "generic_can_hardware_interface.hpp"

#include "can_topic_routing.h"

#include <cstring>
#include <stdexcept>



// Constructor
CanNode::CanNode() : Node("generic_can_hardware_interface")
{
    // Parameters (all of these live in config/can_loop.yaml)
    loop_name                = this->declare_parameter<std::string>("loop_name", "unnamed");
    can_interface            = this->declare_parameter<std::string>("can_interface", "can0");
    enable_can_fd            = this->declare_parameter<bool>("enable_can_fd", false);
    mirror_typed_to_incoming = this->declare_parameter<bool>("mirror_typed_to_incoming", false);
    max_frames_per_poll      = this->declare_parameter<int>("max_frames_per_poll", 32);
    const double rx_poll_rate_hz = this->declare_parameter<double>("rx_poll_rate_hz", 100.0);
    const int qos_depth          = this->declare_parameter<int>("qos_depth", 5);

    max_payload_bytes = enable_can_fd ? CANFD_MAX_DLEN : CAN_MAX_DLEN;
    topic_base = "/can/" + loop_name + "/";

    // Reliable, keep last N. CAN traffic is small and we would rather queue a
    // command than silently drop it.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth));

    incoming_pub = this->create_publisher<GenericCommsMsg>(topic_base + "incoming", qos);
    create_typed_publishers(qos);

    outgoing_sub = this->create_subscription<GenericCommsMsg>(
        topic_base + "outgoing", qos,
        std::bind(&CanNode::outgoing_callback, this, std::placeholders::_1));

    if (!can_bus.open(can_interface, enable_can_fd)) {
        RCLCPP_FATAL(this->get_logger(), "Could not open CAN interface '%s': %s",
                     can_interface.c_str(), can_bus.last_error().c_str());
        RCLCPP_FATAL(this->get_logger(), "Is the interface up? Try scripts/can_setup.sh");
        throw std::runtime_error("failed to open CAN interface");
    }

    rx_timer = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rx_poll_rate_hz),
        std::bind(&CanNode::rx_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "CAN loop '%s' up on %s (%s), topics under %s",
                loop_name.c_str(), can_interface.c_str(),
                enable_can_fd ? "CAN FD" : "classic CAN", topic_base.c_str());
}


// Main function (entry point of node)
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // The constructor throws if the interface is not there. Catch it so we exit
    // with a readable message instead of a raw terminate().
    try {
        auto node = std::make_shared<CanNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("generic_can_hardware_interface"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}


// Member function implementations (sub and timer callbacks go first)

// Anything published on /can/<loop_name>/outgoing gets written straight to the bus.
void CanNode::outgoing_callback(const GenericCommsMsg::SharedPtr msg)
{
    if (msg->payload.size() > max_payload_bytes) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), ERROR_LOG_PERIOD_MS,
                              "Dropping message type %u: payload is %zu bytes, max is %zu",
                              msg->type, msg->payload.size(), max_payload_bytes);
        return;
    }

    // payload_len is redundant with the array size, but comms uses it so keep
    // an eye on it. The array is what actually gets sent.
    if (msg->payload_len != msg->payload.size()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), ERROR_LOG_PERIOD_MS,
                             "Message type %u has payload_len %u but %zu payload bytes, using %zu",
                             msg->type, msg->payload_len, msg->payload.size(), msg->payload.size());
    }

    struct canfd_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = pack_id(msg->sender_id, msg->type) | CAN_EFF_FLAG;  // always extended ids
    frame.len = (uint8_t)msg->payload.size();
    std::memcpy(frame.data, msg->payload.data(), frame.len);

    if (!can_bus.write_frame(frame)) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), ERROR_LOG_PERIOD_MS,
                              "CAN write failed on %s: %s",
                              can_interface.c_str(), can_bus.last_error().c_str());
    }
}

// Drains the socket. The socket is non blocking, so this returns as soon as
// there is nothing left to read.
void CanNode::rx_timer_callback()
{
    for (int i = 0; i < max_frames_per_poll; i++) {
        struct canfd_frame frame;
        bool no_data = false;

        if (!can_bus.read_frame(&frame, &no_data)) {
            if (no_data) return;   // socket is empty, we are done for this cycle
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), ERROR_LOG_PERIOD_MS,
                                  "CAN read failed on %s: %s",
                                  can_interface.c_str(), can_bus.last_error().c_str());
            return;
        }

        // Error frames are the driver telling us about the bus, not a comms message.
        if (frame.can_id & CAN_ERR_FLAG) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), ERROR_LOG_PERIOD_MS,
                                 "CAN error frame on %s (id 0x%08X)",
                                 can_interface.c_str(), frame.can_id);
            continue;
        }
        // Remote transmission requests are unused by comms.
        if (frame.can_id & CAN_RTR_FLAG) continue;
        // comms always uses 29 bit ids, so a standard id is somebody else's traffic.
        if (!(frame.can_id & CAN_EFF_FLAG)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), ERROR_LOG_PERIOD_MS,
                                 "Ignoring standard id frame 0x%03X on %s",
                                 frame.can_id & CAN_SFF_MASK, can_interface.c_str());
            continue;
        }

        const uint32_t id = frame.can_id & CAN_EFF_MASK;


        GenericCommsMsg msg;
        msg.sender_id   = id_device(id);
        msg.type        = id_type(id);
        msg.payload_len = frame.len;
        msg.payload.assign(frame.data, frame.data + frame.len);

        publish_incoming(msg);
        

    }
}

// Walks every message type once at startup and creates a publisher for the ones
// that asked for their own topic.
void CanNode::create_typed_publishers(const rclcpp::QoS& qos)
{
    for (uint16_t type = 0; type < (uint16_t)NUM_MSG_TYPES; type++) {
        const char* suffix = type_to_topic_suffix(type);
        if (suffix == nullptr) continue;

        const std::string topic = topic_base + suffix;
        typed_incoming_pubs[type] = this->create_publisher<GenericCommsMsg>(topic, qos);
        RCLCPP_INFO(this->get_logger(), "Message type %u routed to %s", type, topic.c_str());
    }
}

// Typed topic if the type has one, generic topic otherwise.
void CanNode::publish_incoming(const GenericCommsMsg& msg)
{
    auto pub = typed_incoming_pubs.find(msg.type);
    if (pub != typed_incoming_pubs.end()) {
        pub->second->publish(msg);
        if (!mirror_typed_to_incoming) return;
    }
    incoming_pub->publish(msg);
}