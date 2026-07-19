#include "death_ray_magnetometer.h"

#include <fcntl.h>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>

DeathRayMagnetometerNode::DeathRayMagnetometerNode() : Node("death_ray_magnetometer_node") {
    if (initSerial()) {
        RCLCPP_INFO(this->get_logger(), "Successfully connected to IMU on %s", SERIAL_DEVICE_NAME);
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s!", SERIAL_DEVICE_NAME);
    }
    
    heading_pub_ = this->create_publisher<std_msgs::msg::Float32>("death_ray_heading", 10);

    heading_feedback_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(HEADING_FEEDBACK_PUBLISH_FREQUENCY_MS),
        std::bind(&DeathRayMagnetometerNode::publishHeadingFeedback, this)
    );
}

DeathRayMagnetometerNode::~DeathRayMagnetometerNode() {
    if (uart_connection >= 0) {
        RCLCPP_INFO(this->get_logger(), "Closing serial port connection to IMU.");
        close(uart_connection);
    }
}

bool DeathRayMagnetometerNode::initSerial() {
    /**
     * Open UART connection to the IMU using the `open` syscall (like a file)
     * 
     * `O_RDWR`: Open for reading and writing. Normal operation of the node only
     *           requires reading, but writing is needed for triggering calibration 
     *           mode from the terminal.
     * `O_NDELAY`: Open in non-blocking mode without waiting for handshake, as the
     *             IMU can't handshake
     */
    uart_connection = open(SERIAL_DEVICE_NAME, O_RDWR | O_NDELAY);
    if (uart_connection < 0) {
        return false;
    }

    /**
     * After opening the connection in non-blocking mode, switch to
     * blocking mode to avoid polling when there is no data on the serial line.
     */
    fcntl(uart_connection, F_SETFL, 0);

    /*****************************************************************/
    /************************* UART Settings *************************/
    /*****************************************************************/
    struct termios tty;
    if (tcgetattr(uart_connection, &tty) != 0) {
        return false;
    }

    cfsetospeed(&tty, B57600); // Output speed
    cfsetispeed(&tty, B57600); // Input speed

    /**
     * Exclude the parity bit, since one corrupted message from UART is unlikely to 
     * impact the overall dish rotation system.
     */
    tty.c_cflag &= ~PARENB;

    tty.c_cflag &= ~CSTOPB; // 1 stop-bit at the end of each packet

    // Send one byte of data at a time
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag |= (CLOCAL | CREAD); // Enable the Pi RX pin for reading

    // Canonical mode: to read bytes as they arrive instead of waiting for a newline
    tty.c_lflag &= ~ICANON;

    /**
     * Reduce clutter on the UART line by preventing the Pi from transmitting
     * the data it receives back to the IMU.
     */
    tty.c_lflag &= ~(ECHO | ECHOE);

    /**
     * Disable signal generation, so control characters in the data stream don't
     * control the terminal and/or crash the ROS node.
     */
    tty.c_lflag &= ~ISIG;

    /**
     * Enable Raw Output Mode, so when the Pi sends data to the IMU (i.e. only during
     * calibration), it sends the raw bytes without processing
     */
    tty.c_oflag &= ~OPOST;

    // Set a 0.1s timeout for attempts to read from the `uart_connection` file
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    // Commit the updated UART settings
    if (tcsetattr(uart_connection, TCSANOW, &tty) != 0) {
        return false;
    }

    /*****************************************************************/
    /*********************** End UART Settings ***********************/
    /*****************************************************************/

    return true;
}

void DeathRayMagnetometerNode::publishHeadingFeedback() {
    if (uart_connection >= 0) {
        char buf[256];
        int bytes_read = read(uart_connection, &buf, sizeof(buf) - 1);

        if (bytes_read > 0) {
            buf[bytes_read] = '\0';
            read_buffer += std::string(buf);

            size_t newline_pos;

            /**
             * Repeat as long as the buffer contains a newline character.
             * Otherwise, there is nothing new to publish on this iteration
             * of the publisher loop.
             */
            while ((newline_pos = read_buffer.find('\n')) != std::string::npos) {
                std::string line = read_buffer.substr(0, newline_pos);
                read_buffer.erase(0, newline_pos + 1);

                /**
                 * The IMU firmware ends every line with CRLF. The LF
                 * is already removed in the `substr` call above, but
                 * still need to remove the CR to be left with data only.
                 */
                if (!line.empty() && line.back() == '\r') {
                    line.pop_back();
                }

                if (line.rfind(IMU_PACKET_PREFIX, 0) == 0) {
                    try {
                        std::string data = line.substr(sizeof(IMU_PACKET_PREFIX));
                        std::stringstream stream(data);

                        std::string heading_str;

                        if (std::getline(stream, heading_str, ',')) {
                            float heading = std::stof(heading_str);

                            heading_buffer.push(heading);
                            if (heading_buffer.size() > MOVING_AVERAGE_FILTER_SIZE) {
                                heading_buffer.pop();
                            }

                            /**
                             * Simply averaging the angles in the queue would be incorrect
                             * For example, if heading1 = 179 and heading2 = -179, their naive average 
                             * would be 0 even though in our domain the angles are only 2 degrees apart
                             * 
                             * Instead, average the sin and cos of the angles, and calculate the final
                             * average using arctan. In the example above, this would give 180 degrees
                             * as desired
                             */
                            double sum_sin = 0.0;
                            double sum_cos = 0.0;
                            std::queue<float> temp_queue = heading_buffer;
                            
                            while (!temp_queue.empty()) {
                                double rad = temp_queue.front() * M_PI / 180.0;
                                sum_sin += std::sin(rad);
                                sum_cos += std::cos(rad);
                                temp_queue.pop();
                            }

                            double avg_rad = std::atan2(sum_sin, sum_cos);
                            float filtered_heading = avg_rad * 180.0 / M_PI;
                            if (filtered_heading < 0.0f) {
                                filtered_heading += 360.0f;
                            }

                            std_msgs::msg::Float32 message;
                            message.data = filtered_heading;
                            heading_pub_->publish(message);
                        }
                    } catch (const std::exception& e) {
                        RCLCPP_WARN(this->get_logger(), "Failed to parse serial line: %s", e.what());
                    }
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
