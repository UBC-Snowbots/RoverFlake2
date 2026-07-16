#ifndef DEATH_RAY_MAGNETOMETER_H
#define DEATH_RAY_MAGNETOMETER_H

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#define HEADING_FEEDBACK_PUBLISH_FREQUENCY_MS 200
#define SERIAL_DEVICE_NAME "/dev/serial0"

/**
 * @brief DeathRayMagnetometerNode reads the direction the death ray is facing
 * and publishes it
 */
class DeathRayMagnetometerNode : public rclcpp::Node {
public:
    DeathRayMagnetometerNode();
    ~DeathRayMagnetometerNode();

private:
    bool initSerial();
    void publishHeadingFeedback();
    
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr heading_pub_;
    rclcpp::TimerBase::SharedPtr heading_feedback_timer_;

    int serial_connection = -1;
    std::string rx_buffer_ = "";
};

#endif
