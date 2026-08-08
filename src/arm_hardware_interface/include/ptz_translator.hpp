// Bridges /ptz/control (geometry_msgs/Vector3) onto the generic CAN loop as a
// MSG_TYPE__PTZ_VEL_CMD. See generic_can_hardware_interface.hpp for the loop.
#pragma once

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rover_msgs/msg/generic_comms_msg.hpp>  

using GenericCommsMsg = rover_msgs::msg::GenericCommsMsg;
using Vector3         = geometry_msgs::msg::Vector3;

// Log throttle period, matches the CAN node.
static constexpr int ERROR_LOG_PERIOD_MS = 1000;

class PtzControlNode : public rclcpp::Node
{
public:
    PtzControlNode();

private:
    // Callbacks
    void control_callback(const Vector3::SharedPtr msg);

    // Helpers
    void send_ptz_vel_cmd(double vel_pan, double vel_tilt);

    // Parameters
    std::string control_topic;
    std::string outgoing_topic;
    uint16_t    device_id        = 0;
    double      max_speed        = 100.0;   
    double      deadband         = 0.0;     // inputs below this are treated as 0
    bool        invert_pan       = false;
    bool        invert_tilt      = false;
    double      tx_rate_hz       = 20.0;    // republish rate, must beat the firmware watchdog
    double      input_timeout_s  = 0.5;     // no input for this long -> stop the servos

    // State
    double last_vel_pan  = 0.0;
    double last_vel_tilt = 0.0;
    rclcpp::Time last_rx_time;
    bool  have_input   = false;   // have we ever received a command
    bool  input_active = false;   // is the publisher currently considered alive

    // ROS handles
    rclcpp::Subscription<Vector3>::SharedPtr      control_sub;
    rclcpp::Publisher<GenericCommsMsg>::SharedPtr outgoing_pub;
    rclcpp::TimerBase::SharedPtr                  tx_timer;
};