// PTZ control -> CAN bridge. Subscribes to /ptz/control and emits an encoded
// MSG_TYPE__PTZ_VEL_CMD on /can/<loop>/outgoing for the STM32 to pick up.

// This node should not have any fancy pants logic. Just take the inputs and pipe them on the can bus
#include <ptz_translator.hpp>

#include <algorithm>
#include <cmath>
#include <type_traits>

// comms.h is C. Drop the extern "C" if the header already guards itself.
extern "C" {
#include "comms.h"
#include "device_ids.h"
}

#define DEBUG_MSGS // if defined, print debug msgs

// Constructor
PtzControlNode::PtzControlNode() : Node("ptz_control_node")
{
    control_topic   = this->declare_parameter<std::string>("control_topic", "/ptz/control");
    outgoing_topic  = this->declare_parameter<std::string>("outgoing_topic", "/can/main/outgoing");
    device_id       = (uint16_t)this->declare_parameter<int>("device_id", 33); //TODO deshittify
    max_speed       = this->declare_parameter<double>("max_speed", 100.0);
    deadband        = this->declare_parameter<double>("deadband", 0.0);
    invert_pan      = this->declare_parameter<bool>("invert_pan", false);
    invert_tilt     = this->declare_parameter<bool>("invert_tilt", false);
    input_timeout_s = this->declare_parameter<double>("input_timeout_s", 0.5);
    const int qos_depth = this->declare_parameter<int>("qos_depth", 15);

    if (device_id == 0) {
        RCLCPP_WARN(this->get_logger(),
                    "device_id is 0, set it to this node's id from device_ids.h");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth));

    outgoing_pub = this->create_publisher<GenericCommsMsg>(outgoing_topic, qos);

    // Best effort on the input side: a stale joystick sample is worse than a
    // dropped one, and the tx timer republishes anyway.
    control_sub = this->create_subscription<Vector3>(
        control_topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        std::bind(&PtzControlNode::control_callback, this, std::placeholders::_1));


    RCLCPP_INFO(this->get_logger(), "PTZ bridge up: %s -> %s (device id %u, %.1f Hz)",
                control_topic.c_str(), outgoing_topic.c_str(), device_id, tx_rate_hz);
}


// Main function (entry point of node)
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PtzControlNode>());
    rclcpp::shutdown();
    return 0;
}

// Dumb input output.
void PtzControlNode::control_callback(const Vector3::SharedPtr msg)
{
    send_ptz_vel_cmd(msg->x, msg->y);
}

// Builds a msg_t, runs it through the comms encoder, and hands the raw payload
// to the CAN node. Using encode() rather than packing bytes by hand keeps this
// in step with comms.h if the struct ever changes.
void PtzControlNode::send_ptz_vel_cmd(double vel_pan, double vel_tilt)
{
    msg_t msg = {};
    msg.type = MSG_TYPE__PTZ_VEL_CMD;

    msg.payload.ptz_vel_cmd.vel_pan  = vel_pan;
    msg.payload.ptz_vel_cmd.vel_tilt = vel_tilt;

    comms_set_device_id(device_id);
    encoded_msg_t encoded = {};
    bool rc = encode(&msg, &encoded);
    
    GenericCommsMsg out;
    out.sender_id   = device_id;
    out.type        = (uint16_t)msg.type;
    out.payload_len = encoded.payload_len;
    out.payload.resize(8);
    out.payload.assign(encoded.payload, encoded.payload + encoded.payload_len);
    
    outgoing_pub->publish(out);
    #ifdef DEBUG_MSGS
    RCLCPP_INFO(this->get_logger(), "encode rc=%d payload_len=%u", rc, encoded.payload_len);
    RCLCPP_INFO(this->get_logger(), "Got command, tilt: %f, pan: %f", vel_tilt, vel_pan);
    RCLCPP_INFO(this->get_logger(), "Raw packed bytes: %i %i %i %i %i %i %i %i", out.payload[0], out.payload[1], out.payload[2], out.payload[3], out.payload[4], out.payload[5], out.payload[6], out.payload[7]);
    #endif
}