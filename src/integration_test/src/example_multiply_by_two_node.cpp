#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/int32.hpp> 

// This node is just an example node to show integration tests
// Its just one publisher and one subscriber, give it an integer and it just doubles it

// Class definition
class MultiplyBy2Node : public rclcpp::Node {
public:
    MultiplyBy2Node(); // Constructor is defined in .cpp.

private:
    // SUBS AND PUBS
    rclcpp::Publisher<example_interfaces::msg::Int32>::SharedPtr output_pub;
    rclcpp::Subscription<example_interfaces::msg::Int32>::SharedPtr input_sub;

    // Callbacks
    void test_callback(example_interfaces::msg::Int32 msg);

};    

// Constructor
MultiplyBy2Node::MultiplyBy2Node() : Node("multiply_by_two_integration_test_example_node") 
{ // We create a class using rclcpp::Node as a base class. You can still use another base class if you need, albeit sometimes with difficulties in passing args..
   
    // Quality of service example
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

    output_pub = this->create_publisher<example_interfaces::msg::Int32>(
        "/integration/test_int/output", qos);
    
    input_sub = this->create_subscription<example_interfaces::msg::Int32>(
        "/integration/test_int/input", qos, std::bind(&MultiplyBy2Node::test_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Example Log message: Node has started");
}

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiplyBy2Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

void MultiplyBy2Node::test_callback(example_interfaces::msg::Int32 msg)
{
    example_interfaces::msg::Int32 output;
    output.data = msg.data *2;
    output_pub->publish(output);
}