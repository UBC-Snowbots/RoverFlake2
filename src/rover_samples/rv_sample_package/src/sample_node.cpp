//A little blurb here or in your header is always welcome
#include "sample_node.h"

// Constructor
SampleNode::SampleNode() : Node("sample_node") 
{ // We create a class using rclcpp::Node as a base class. You can still use another base class if you need, albeit sometimes with difficulties in passing args..
   
    // Quality of service example
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

    joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", qos /* or an int for quick qos*/, std::bind(&SampleNode::joy_callback, this, std::placeholders::_1));
    
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", qos);
        
        timer_ = this->create_wall_timer(
        std::chrono::duration<double>(TIMER_PERIOD_MS),std::bind(&SampleNode::timer_callback, this));

    // parameters:
    this->declare_parameter("param_name", std::vector<std::string>({"Watchdog"}));

}


// Main function (entry point of node)
int main(int argc, char *argv[]) 
{
    // Init ros2 with args. 
    rclcpp::init(argc, argv);

    // Instatiate your node
    auto node = std::make_shared<SampleNode>();
    // Spin it! (this is what runs pubs and subs. This is a blocking function)
    rclcpp::spin(node);
    // There is also rclcpp::spinSome(node) if you need more control over when the node spins / need to not block while spinning.
    
    // Code only reaches here if we crash or shutdown the node
    rclcpp::shutdown();
    return 0;
}

// Member function implementations (sub and timer callbacks go first)

void SampleNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    float steering_angle = msg->axes[0];
    float speed = 99.0 - (msg->axes[3] + 1)*100;
    float acceleration = (msg->axes[2] + 1)*20 - (msg->axes[3] + 1)*20;
    float jerk = (msg->axes[2] + 1)*20 - (msg->axes[3] + 1)*20;
    int paddleR = msg->buttons[4];
    int paddleL = msg->buttons[5];
    send_generic_command(steering_angle, speed, acceleration, jerk);
    
}

void SampleNode::timer_callback()
{
    // Debug messages are good to wrap in a compiler instruction - Then you can turn on / turn off debug messages easily. For comp, we don't want every node to be spamming info as it is uneeded overhead
    #ifdef DEBUG_MSGS
    std::cout << "Node is running (std cout)" << std::endl; // Standard C++ is always available
    RCLCPP_INFO(this->get_logger(), "Node is running (ros2 info)"); // ROS2 (rclcpp) also has functions/macros that can replace some of the standard c++ workflow.
    RCLCPP_WARN(this->get_logger(), "Node is running (ros2 warn)"); // ROS2 (rclcpp) also has functions that can replace some of the standard c++ workflow.
    RCLCPP_ERROR(this->get_logger(), "Node is running (ros2 error)"); // ROS2 (rclcpp) also has functions that can replace some of the standard c++ workflow.
    #endif // DEBUG_MSGS
}


void SampleNode::send_generic_command(float steering_angle, float speed, float acceleration, float jerk) 
{
    // msg.steering angle = steering angle;
    // etc...
    // some_publisher.pub(msg); 
}







// Other examples/tips

//params can be refreshed at runtime
// std::vector<std::string> expected_nodes = this->get_parameter("expected_nodes").as_string_array();


