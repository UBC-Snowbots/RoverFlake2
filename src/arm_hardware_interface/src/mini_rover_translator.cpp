// TEMPORARY: /cmd_vel (geometry_msgs/Twist) -> MSG_TYPE__MINI_ROVER_VEL_CMD on /can/<loop>/outgoing.
// Same pattern as ptz_translator. No fancy logic, just pipe twist onto the bus.
//
// The firmware stops the motors if it gets no command for 500 ms, so the last command is
// republished at tx_rate_hz while input is fresh. When input goes stale, one zero command is sent.

#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rover_msgs/msg/generic_comms_msg.hpp>

extern "C" {
#include "comms.h"
#include "device_ids.h"
}

using GenericCommsMsg = rover_msgs::msg::GenericCommsMsg;
using Twist           = geometry_msgs::msg::Twist;

class MiniRoverTranslator : public rclcpp::Node
{
public:
    MiniRoverTranslator() : Node("mini_rover_translator")
    {
        cmd_vel_topic   = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        outgoing_topic  = declare_parameter<std::string>("outgoing_topic", "/can/main/outgoing");
        device_id       = (uint16_t)declare_parameter<int>("device_id", DEVICE_ID__ON_BOARD_NUC_NUBS); // sender id (this computer)
        tx_rate_hz      = declare_parameter<double>("tx_rate_hz", 20.0);
        input_timeout_s = declare_parameter<double>("input_timeout_s", 0.5);

        comms_set_device_id(device_id);

        outgoing_pub = create_publisher<GenericCommsMsg>(outgoing_topic, rclcpp::QoS(10));
        cmd_vel_sub  = create_subscription<Twist>(
            cmd_vel_topic, rclcpp::QoS(10),
            [this](const Twist::SharedPtr msg) {
                lin_mps   = msg->linear.x;
                ang_degps = msg->angular.z * 180.0 / M_PI; // cmd_vel is rad/s, comms msg is deg/s
                last_rx   = now();
                active    = true;
                send(lin_mps, ang_degps);
            });

        tx_timer = create_wall_timer(
            std::chrono::duration<double>(1.0 / tx_rate_hz),
            [this]() {
                if (!active) return;
                if ((now() - last_rx).seconds() > input_timeout_s) {
                    active = false;
                    send(0.0, 0.0);
                    RCLCPP_INFO(get_logger(), "cmd_vel stale, sent stop");
                    return;
                }
                send(lin_mps, ang_degps);
            });

        RCLCPP_INFO(get_logger(), "Mini rover bridge up: %s -> %s (sender id 0x%04X, %.1f Hz)",
                    cmd_vel_topic.c_str(), outgoing_topic.c_str(), device_id, tx_rate_hz);
    }

private:
    void send(double lin, double ang)
    {
        msg_t msg = {};
        msg.type = MSG_TYPE__MINI_ROVER_VEL_CMD;
        msg.payload.mini_rover_velcmd.lin_vel_mps   = (float)lin;
        msg.payload.mini_rover_velcmd.ang_vel_degps = (float)ang;

        encoded_msg_t encoded = {};
        if (!encode(&msg, &encoded)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "encode() failed");
            return;
        }

        GenericCommsMsg out;
        out.sender_id   = device_id;
        out.type        = (uint16_t)msg.type;
        out.payload_len = encoded.payload_len;
        out.payload.assign(encoded.payload, encoded.payload + encoded.payload_len);
        outgoing_pub->publish(out);
    }

    std::string cmd_vel_topic, outgoing_topic;
    uint16_t    device_id = 0;
    double      tx_rate_hz = 20.0, input_timeout_s = 0.5;

    double       lin_mps = 0.0, ang_degps = 0.0;
    rclcpp::Time last_rx;
    bool         active = false;

    rclcpp::Subscription<Twist>::SharedPtr        cmd_vel_sub;
    rclcpp::Publisher<GenericCommsMsg>::SharedPtr outgoing_pub;
    rclcpp::TimerBase::SharedPtr                  tx_timer;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MiniRoverTranslator>());
    rclcpp::shutdown();
    return 0;
}