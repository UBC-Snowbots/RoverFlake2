#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/arm_command.hpp>
#include <serial/serial.h>
#include <string>
#include <vector>
#include <sstream>

#define NUM_JOINTS 6
#define ABS_VEL_CMD 'V'

class SerialPublisher : public rclcpp::Node
{
public:
    SerialPublisher() : Node("arduino_serial_publisher")
    {
        publisher_ = this->create_publisher<rover_msgs::msg::ArmCommand>("/arm/command", 10);

        std::string port = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0";
        unsigned long baud = 9600;
        try
        {
            serial_ = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));
            if (serial_->isOpen())
            {
                RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully", port.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", port.c_str());
            }
        }
        catch (const serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "IOException: %s", e.what());
        }

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&SerialPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        //arduino serial should publish 6 pot values seperated by comma
        if (serial_->available())
        {
            std::string line = serial_->readline();
            std::stringstream ss(line);
            std::string item;
            std::vector<double> values;

            while (std::getline(ss, item, ','))
            {
                try
                {
                    values.push_back(std::stod(item));
                }
                catch (const std::invalid_argument &e)
                {
                    RCLCPP_WARN(this->get_logger(), "Invalid argument: %s", e.what());
                }
                catch (const std::out_of_range &e)
                {
                    RCLCPP_WARN(this->get_logger(), "Out of range: %s", e.what());
                }
            }

            if (values.size() == 6)  // Assuming 3 pots per joystick, total of 6 pots
            {
                auto msg = std::make_shared<rover_msgs::msg::ArmCommand>();
                msg->cmd_type = ABS_VEL_CMD;  // Set appropriate command type
                msg->velocities = values;     // Populate velocities from joystick values

                publisher_->publish(*msg);
                RCLCPP_INFO(this->get_logger(), "Published velocities: [%f, %f, %f, %f, %f, %f]",
                            msg->velocities[0], msg->velocities[1], msg->velocities[2], msg->velocities[3], msg->velocities[4], msg->velocities[5]);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Expected 6 values, but got %zu", values.size());
            }
        }
    }

    rclcpp::Publisher<rover_msgs::msg::ArmCommand>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    serial::Serial *serial_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialPublisher>());
    rclcpp::shutdown();
    return 0;
}
