#include <chrono>
// #include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <iostream>
#include <thread>
#include <serial/serial.h>

using namespace std;
using namespace std::chrono_literals;

class JointStatePublisher : public rclcpp::Node
{
public:
    JointStatePublisher() : Node("joint_state_publisher")
    {
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

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        tim = this->create_wall_timer(500ms, std::bind(&JointStatePublisher::publish_joint_states, this));
    }

private:
    void publish_joint_states()
    {
        // if (serial_->available())
        // {
        //     std::string line = serial_->readline();
        //     cout << "line: " << line << endl;

        //     // size_t commaPos = line.find(',');

        //     std::stringstream ss(line);
        //     std::string token;
        //     int index = 0;

        //     auto message = sensor_msgs::msg::JointState();
        //     message.header.stamp = this->get_clock()->now();
        //     message.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

        //     // Parse each comma-separated value
        //     while (std::getline(ss, token, ',') && index < 6) {
        //         message.position[index] = std::stod(token);  // Convert the token to an integer
        //         index++;
        //     }

        //     publisher_->publish(message);

            // if (commaPos != std::string::npos) {
            //     // Extract substrings before and after the comma
            //     // std::string value1Str = line.substr(0, commaPos);
            //     // std::string value2Str = line.substr(commaPos + 1);

            //     // // Convert the strings to integers (or floats)
            //     // int value1 = std::stoi(value1Str);
            //     // // value2 = std::stoi(value2Str);
            //     // double value2 = std::stod(line); 

            //     try {
            //         // double value = std::stod(line); // Convert string to double
            //         int value1 = std::stoi(value1Str);
            //         double value2 = std::stod(line); 

            //         std::cout << "Value 1: " << value1 << std::endl;
            //         std::cout << "Value 2: " << value2 << std::endl;

            //         // std::cout << "Converted value: " << value << std::endl;
                
            //         auto message = sensor_msgs::msg::JointState();
            //         message.header.stamp = this->get_clock()->now();
                    
            //         // message.name = {"base_to_l3", "l3_to_l2", "l2_to_l1", "l1_to_grip"};
            //         // message.position = {value, 1.5, 1.0, 1.0};  // Joint positions

            //         message.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
            //         message.position = {value, 1.0, 1.0, 1.0, 1.0, 1.0};  // Joint positions

            //         for (size_t i = 0; i < count; i++)
            //         {
            //             /* code */
            //         }
                    

            //         publisher_->publish(message);

            //     } catch (const std::invalid_argument& e) {
            //         std::cerr << "Invalid argument: " << e.what() << std::endl;
            //     } catch (const std::out_of_range& e) {
            //         std::cerr << "Out of range: " << e.what() << std::endl;
            //     }

            // } else {
            //     std::cout << "Invalid input format. Please enter two values separated by a comma." << std::endl;
            // }

        // }

        if (serial_->available())
        {
            std::string line = serial_->readline();
            std::cout << "line: " << line << std::endl;

            std::stringstream ss(line);
            std::string token;
            int index = 0;

            auto message = sensor_msgs::msg::JointState();
            message.header.stamp = this->get_clock()->now();
            message.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

            // Ensure the position vector has space for 6 joints
            // message.position.resize(6);

            // Parse each comma-separated value
            while (std::getline(ss, token, ',') && index < 6) {
                try {
                    double d = std::stod(token);
                    std::cout << d << std::endl;
                    // message.position[index] = d;  // Convert the token to a double
                    message.position.push_back(d);
                } catch (const std::invalid_argument& e) {
                    std::cerr << "Invalid number: " << token << std::endl;
                    return;  // Handle the error gracefully
                } catch (const std::out_of_range& e) {
                    std::cerr << "Number out of range: " << token << std::endl;
                    return;  // Handle the error gracefully
                }
                index++;
            }

            // Publish the joint state message
            publisher_->publish(message);
        }

        // auto message = sensor_msgs::msg::JointState();
        // message.header.stamp = this->get_clock()->now();
        
        // // Define joint names and positions (example with two joints)
        // message.name = {"base_to_l3", "l3_to_l2", "l2_to_l1", "l1_to_grip"};
        // message.position = {p1, 1.5, 1.0, 1.0};  // Joint positions
        // p1 += 0.1;

        // message.velocity = {0.0, 0.0};  // Joint velocities
        // message.effort = {0.0, 0.0};    // Joint efforts

        // publisher_->publish(message);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr tim;
    serial::Serial *serial_;
    double p1 = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePublisher>());
    cout << "spin done" << endl;
    rclcpp::shutdown();
    return 0;
}

// #include <chrono>
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>
// #include <iostream>
// #include <thread>
// #include <string>

// using namespace std;
// using namespace std::chrono_literals;

// class JointStatePublisher : public rclcpp::Node 
// {
// public:
//     JointStatePublisher() : Node("joint_state_publisher")
//     {
//         publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
//         tim = this->create_wall_timer(500ms, std::bind(&JointStatePublisher::publish_joint_states, this));
//     }

// private:
//     void publish_joint_states()
//     {
//         // Read input from the terminal
//         std::string line;
//         std::cout << "Enter joint value: ";
//         std::getline(std::cin, line); // Read a line from standard input

//         // Process the input
//         try {
//             double value = std::stod(line); // Convert string to double

//             std::cout << "Converted value: " << value << std::endl;
            
//             auto message = sensor_msgs::msg::JointState();
//             message.header.stamp = this->get_clock()->now();
            
//             message.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
//             message.position = {value, 1.0, 1.0, 1.0, 1.0, 1.0};  // Joint positions

//             publisher_->publish(message);
//         } catch (const std::invalid_argument& e) {
//             std::cerr << "Invalid argument: " << e.what() << std::endl;
//         } catch (const std::out_of_range& e) {
//             std::cerr << "Out of range: " << e.what() << std::endl;
//         }
//     }

//     rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
//     rclcpp::TimerBase::SharedPtr tim;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<JointStatePublisher>());
//     rclcpp::shutdown();
//     return 0;
// }