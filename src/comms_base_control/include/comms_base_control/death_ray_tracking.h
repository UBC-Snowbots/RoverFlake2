#ifndef DEATH_RAY_TRACKING_H
#define DEATH_RAY_TRACKING_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <cmath>
#include <chrono>

#define MAX_ROTATION 5.0
#define LOOP_RATE_MS 100

/**
 * @brief DeathRayTracking tracks the position of a rover using GNSS coordinates and magnetometer feedback.
 * 
 * Publishes rotation commands to the DeathRayMotorControlNode based on the rover's position relative to the comms base.
 */
class DeathRayTracking : public rclcpp::Node {
public:
    DeathRayTracking();
    ~DeathRayTracking();

private:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr death_ray_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr rover_gnss_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr comms_gnss_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr magnometer_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr active_tracking_sub_;
    rclcpp::TimerBase::SharedPtr tracking_timer_;
    double comms_latitude_;
    double comms_longitude_;
    double rover_latitude_;
    double rover_longitude_;
    float magnetometer_feedback_;
    bool tracking_active_;
    bool rover_has_fix_;
    bool comms_has_fix_;

    void roverGnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void commsGnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void magnetometerCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void activeTrackingCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void trackingLoop();
    double degToRad(double degrees);
    double radToDeg(double radians);
    double normalizeAngle(double angle);
    double shortestAngularDistance(double target, double current);
};


#endif // DEATH_RAY_TRACKING_H