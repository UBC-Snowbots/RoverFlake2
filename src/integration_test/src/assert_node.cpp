// name?

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp> 
#include <example_interfaces/msg/int32.hpp> 
#include <catch2/catch_all.hpp>

// Class definition
class IntegrationTestNode : public rclcpp::Node {
public:
    IntegrationTestNode(); // Constructor is defined in .cpp.

private:
    // SUBS AND PUBS
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher;

    rclcpp::Publisher<example_interfaces::msg::Int32>::SharedPtr checker_pub;
    rclcpp::Subscription<example_interfaces::msg::Int32>::SharedPtr checker_sub;

    // Callbacks
    void test_callback(example_interfaces::msg::Int32 msg);

};    

// Constructor
IntegrationTestNode::IntegrationTestNode() : Node("integration_test_node") 
{ // We create a class using rclcpp::Node as a base class. You can still use another base class if you need, albeit sometimes with difficulties in passing args..
   
    // Quality of service example
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

    joy_publisher = this->create_publisher<sensor_msgs::msg::Joy>(
        "/joy", qos);

    checker_pub = this->create_publisher<example_interfaces::msg::Int32>(
        "/integration/test_int", qos);
    
    checker_sub = this->create_subscription<example_interfaces::msg::Int32>(
        "/integration/test_int", qos, std::bind(&IntegrationTestNode::test_callback, this, std::placeholders::_1));

}

// Main function (entry point of node)
int main(int argc, char *argv[]) 
{
    // Init ros2 with args. 
   rclcpp::init(argc, argv);

    // Instatiate your node
    auto node = std::make_shared<IntegrationTestNode>();
    // Spin it! (this is what runs pubs and subs. This is a blocking function)
    rclcpp::spin(node);
    // There is also rclcpp::spinSome(node) if you need more control over when the node spins / need to not block while spinning.
    
    // Code only reaches here if we crash or shutdown the node
    rclcpp::shutdown();
    return 0;
}

void IntegrationTestNode::test_callback(example_interfaces::msg::Int32 msg)
{

}