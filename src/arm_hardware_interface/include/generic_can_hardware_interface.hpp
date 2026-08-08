#pragma once
#include <comms.h>

// Generic CAN hardware interface.
//
// One instance of this node owns one SocketCAN interface (one "loop"). It moves
// raw frames between the bus and ROS2, and does not encode or decode payloads -
// that is the comms library's job, done by whoever is on the other end of the topic.
//
// Topics (loop_name is a ros parameter):
//   /can/<loop_name>/incoming        every frame that has no typed topic
//   /can/<loop_name>/<message_type>  optional per type topics, see can_topic_routing.h
//   /can/<loop_name>/outgoing        anything published here is written to the bus
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rover_msgs/msg/generic_comms_msg.hpp>

#include "socket_can.h"

#include <map>
#include <string>

// How long we wait between error logs when the bus is misbehaving, so a broken
// cable doesn't produce a million log lines a second.
#define ERROR_LOG_PERIOD_MS 1000

class CanNode : public rclcpp::Node {
public:
    CanNode();

private:
    using GenericCommsMsg = rover_msgs::msg::GenericCommsMsg;

    // Parameters
    std::string loop_name;
    std::string can_interface;
    bool enable_can_fd = false;
    bool mirror_typed_to_incoming = false; // Set to true to also send special messages in can_topic_routing.h to generic topic
    int max_frames_per_poll = 64;
    size_t max_payload_bytes = 8;

    std::string topic_base;   // "/can/<loop_name>/"

    // Hardware
    SocketCan can_bus;

    // SUBS AND PUBS
    rclcpp::Publisher<GenericCommsMsg>::SharedPtr incoming_pub;
    // Publishers for message types that get their own topic, keyed by message type.
    std::map<uint16_t, rclcpp::Publisher<GenericCommsMsg>::SharedPtr> typed_incoming_pubs;

    rclcpp::Subscription<GenericCommsMsg>::SharedPtr outgoing_sub;
    void outgoing_callback(const GenericCommsMsg::SharedPtr msg);

    // OTHER ROS2 OBJECTS (Timers)
    rclcpp::TimerBase::SharedPtr rx_timer;
    void rx_timer_callback();

    // Member function prototypes
    void create_typed_publishers(const rclcpp::QoS& qos);
    void publish_incoming(const GenericCommsMsg& msg);
};