// A little blurb at the top of your header or .cpp is always welcome!!
//? Best to keep this blurb in header? probably?

// Includes
// If file is not in this directory, use <>
// If file path is reative to this directory, use "". 
#include <rover_utils/include/roverCommon.h> // roverCommon.h will get you set up with basic ros2 includes as well as our utility libraries. Some parts of the repo
#include "sample_node_defines.h"
// Other ros2 includes seperated by a line
#include "sensor_msgs/msg/joy.hpp" 
// cpp specific or other also seperated by a line
#include <atomic>


// Defines go after includes, but before class definitions
// If you need a lot of defines, create a seperate defines file (see sample_node_defines.h for more info)
#define CONSTANT_ONE 45
#define MACRO(x) 45*x


// Class definition
class SampleNode : public rclcpp::Node {
public:
    SampleNode(); // Constructor is usually defined in .cpp. Its okay if you want to define it here instead. 

    void send_command(float steering_angle, float speed, float acceleration, float jerk) {
        //params can be refreshed at runtime
        std::vector<std::string> expected_nodes = this->get_parameter("expected_nodes").as_string_array();

    }

    void send_gear_command(int gear){
      
    }


private:
    int current_gear = 0;
    int prev_paddleR = 0;
    int prev_paddleL = 0;

    // rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr command_publisher_;
    // rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr g29_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
        float steering_angle = msg->axes[0];
        float speed = 99.0 - (msg->axes[3] + 1)*100;
        float acceleration = (msg->axes[2] + 1)*20 - (msg->axes[3] + 1)*20;
        float jerk = (msg->axes[2] + 1)*20 - (msg->axes[3] + 1)*20;
        int paddleR = msg->buttons[4];
        int paddleL = msg->buttons[5];
        
    
    }
};