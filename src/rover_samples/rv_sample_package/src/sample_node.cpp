//A little blurb here or in your header is always welcome
#include "sample_node.h"

// Constructor
SampleNode::SampleNode() : Node("sample_node") { // We create a class using rclcpp::Node as a base class. You can still use another base class if you need, albeit sometimes with difficulties in passing args..
    // Quality of service example
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

    double period = 1.0/CONTROL_RATE_HZ;
    // timer_ = this->create_wall_timer(
    // std::chrono::duration<double>(period),std::bind(&ManualControlNode::test_send, this));
    g29_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", qos /* or an int for quick qos*/, std::bind(&SampleNode::joy_callback, this, std::placeholders::_1));


    // parameters:
    this->declare_parameter("param_name", std::vector<std::string>({"Watchdog"}));

}


// Main function (entry point of node)
int main(int argc, char *argv[]) {
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