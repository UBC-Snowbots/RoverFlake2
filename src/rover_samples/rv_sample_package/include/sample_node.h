// A little blurb at the top of your header or .cpp is always welcome!!
//? Best to keep this blurb in header? probably?
#pragma once // Tells compiler to only include this header once. When in doubt, add to all headers.
// Includes
// If file is not in this directory, use <>
// If file path is reative to this directory, use "". 
#include <rover_utils/include/roverCommon.h> // roverCommon.h will get you set up with basic ros2 includes as well as our utility libraries. Some parts of the repo
#include "sample_node_defines.h"
// Other ros2 includes seperated by a line
#include <sensor_msgs/msg/joy.hpp> 
#include <geometry_msgs/msg/twist_stamped.hpp>
// cpp specific or other also seperated by a line
#include <atomic>


// Defines go after includes, but before class definitions
// If you need a lot of defines, create a seperate defines file (see sample_node_defines.h for more info)
#define DEBUG_MSGS // Comment to turn off debug messages
#define TIMER_HZ 5
#define MACRO(x) 45*x
#define TIMER_PERIOD_MS (1.0/TIMER_HZ)


// using std::your_mom; //? using goes above the class, after definitions

// Class definition
class SampleNode : public rclcpp::Node {
public:
    SampleNode(); // Constructor is defined in .cpp.

private:
    // Most member structures and functions are private in a custom ros2 node. - However, for ease of testing, 
    // you can make some functions public... or just everything public - goes against best practices but were a student team, not a company past series C

    // Data types and structs go first
    int current_gear = 0;
    int prev_paddleR = 0;
    int prev_paddleL = 0;

    // SUBS AND PUBS
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg); // Sub callback goes underneath subscriber definition

    // OTHER ROS2 OBJECTS (Timers)
    rclcpp::TimerBase::SharedPtr timer_;


    // Member function prototypes
    void timer_callback(); // Timer callbacks should go below timer def.
    void send_generic_command(float steering_angle, float speed, float acceleration, float jerk);
    void send_gear_command(int gear);

};    


// Static inline funcs go here?