#include "death_ray_magnetometer.h"

#include <fcntl.h>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>

DeathRayMagnetometerNode::DeathRayMagnetometerNode() : Node("death_ray_magnetometer_node") {
    heading_pub_ = this->create_publisher<std_msgs::msg::Float32>("death_ray_heading", 10);

    heading_feedback_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(HEADING_FEEDBACK_PUBLISH_FREQUENCY_MS),
        std::bind(&DeathRayMagnetometerNode::publishHeadingFeedback, this)
    );
}

DeathRayMagnetometerNode::~DeathRayMagnetometerNode() {
    if (serial_connection >= 0) {
        RCLCPP_INFO(this->get_logger(), "Closing serial port connection to IMU.");
        close(serial_connection);
    }
}

bool DeathRayMagnetometerNode::initSerial() {
    serial_connection = open(SERIAL_DEVICE_NAME, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_connection < 0) {
        return false;
    }

    fcntl(serial_connection, F_SETFL, 0);

    struct termios tty;
    if (tcgetattr(serial_connection, &tty) != 0) {
        return false;
    }

    cfsetospeed(&tty, B57600);
    cfsetispeed(&tty, B57600);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= (CLOCAL | CREAD);

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(serial_connection, TCSANOW, &tty) != 0) {
        return false;
    }

    return true;
}

void DeathRayMagnetometerNode::publishHeadingFeedback() {
    if (serial_connection < 0) {
        std_msgs::msg::Float32 message;
        message.data = 180.0f; // Default fallback
        heading_pub_->publish(message);
        return;
    }

    char read_buf[256];
    int bytes_read = read(serial_connection, &read_buf, sizeof(read_buf) - 1);

    if (bytes_read > 0) {
        read_buf[bytes_read] = '\0';
        rx_buffer_ += std::string(read_buf);

        size_t newline_pos;
        while ((newline_pos = rx_buffer_.find('\n')) != std::string::npos) {
            std::string line = rx_buffer_.substr(0, newline_pos);
            rx_buffer_.erase(0, newline_pos + 1);

            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }

            if (line.rfind("#YPR=", 0) == 0) {
                try {
                    std::string data = line.substr(5);
                    std::stringstream ss(data);
                    std::string yaw_str;

                    if (std::getline(ss, yaw_str, ',')) {
                        float yaw = std::stof(yaw_str);

                        // Normalize Yaw (-180 to 180) to absolute cardinal degrees (0 to 360)
                        float heading = (yaw >= 0.0f) ? yaw : (360.0f + yaw);

                        std_msgs::msg::Float32 message;
                        message.data = heading;
                        heading_pub_->publish(message);
                    }
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "Failed to parse serial line: %s", e.what());
                }
            }
        }
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeathRayMagnetometerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
