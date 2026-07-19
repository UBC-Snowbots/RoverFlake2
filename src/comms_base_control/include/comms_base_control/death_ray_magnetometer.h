#ifndef DEATH_RAY_MAGNETOMETER_H
#define DEATH_RAY_MAGNETOMETER_H

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <queue>

#define HEADING_FEEDBACK_PUBLISH_FREQUENCY_MS 200
#define SERIAL_DEVICE_NAME "/dev/serial0"
#define IMU_PACKET_PREFIX "#YPR="

/**
 * Magnetometer readings are very sensitive, so use an
 * average of multiple readings to smooth things out
 */
#define MOVING_AVERAGE_FILTER_SIZE 32

/**
 * @brief DeathRayMagnetometerNode reads the direction the death ray is facing
 * and publishes it to /death_ray_feedback
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

    /**
     * A Linux file descriptor representing the serial connection to the IMU.
     * 
     * File descriptors are automatically assigned by the kernel when opening
     * a file and are >= 3 (0-2 are reserved for stdin, stdout, stderr). Thus
     * -1 is used here to represent the state when the UART connection is not 
     * opened yet
     */
    int uart_connection = -1;

    /**
     * Buffer to store incoming serial data from the UART until a packet is complete
     */
    std::string read_buffer = "";

    /**
     * To smooth out the heading readings, queue the `MOVING_AVERAGE_FILTER_SIZE`
     * most recent values and publish their average
     */
    std::queue<float> heading_buffer;
};

#endif
