#ifndef DEATH_RAY_MAGNETOMETER_H
#define DEATH_RAY_MAGNETOMETER_H

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#define HEADING_FEEDBACK_PUBLISH_FREQUENCY_MS 200

/**
 * @brief DeathRayMagnetometerNode reads the direction the death ray is facing
 * and publishes it
 */
class DeathRayMagnetometerNode : public rclcpp::Node {
public:
    DeathRayMagnetometerNode();
    ~DeathRayMagnetometerNode();

private:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr heading_pub_;

    // Function to publish the current heading of the death ray
    void publishHeadingFeedback();

    rclcpp::TimerBase::SharedPtr heading_feedback_timer_;

    float heading = 0.0;
};

#endif
