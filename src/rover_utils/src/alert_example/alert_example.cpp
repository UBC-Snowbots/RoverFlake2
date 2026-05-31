#include "alert_example.h"

AlertExampleNode::AlertExampleNode() : Node("alert_example_node") 
{
    alerter = std::make_unique<AlertRaiser>(this);

    example_alert_bar.severity = AlertSeverity::warn;
    example_alert_bar.message = "EXAMPLE: Many gremlocks detected. Rover defenses nearing their limit!";

    example_alert_foo.severity = AlertSeverity::abort;
    example_alert_foo.message = "EXAMPLE: Significant hardware issue detected, aborting to protect hardware.";

    uint32_t index = 0;
    for(auto& alert : example_arrayed_alert)
    {
        std::stringstream os;
        os << "Arrayed Alert " << index << " EXAMPLE" << std::endl;
        alert.message = os.str();
        alert.severity = AlertSeverity::info;
        alerter->raise(alert);
    }

    alerter->raise(example_alert_bar);


        timer = this->create_wall_timer(
        std::chrono::duration<double>(4), std::bind(&AlertExampleNode::timer_callback, this));
}

// Main function (entry point of node)
int main(int argc, char *argv[]) 
{
    // Init ros2 with args. 
    rclcpp::init(argc, argv);

    // Instatiate your node
    auto node = std::make_shared<AlertExampleNode>();
    // Spin it! (this is what runs pubs and subs. This is a blocking function)
    rclcpp::spin(node);
    // There is also rclcpp::spinSome(node) if you need more control over when the node spins / need to not block while spinning.
    
    // Code only reaches here if we crash or shutdown the node
    rclcpp::shutdown();
    return 0;
}

void AlertExampleNode::timer_callback()
{
    // Example with stringstream.
    static uint32_t raises = 0;
    raises++;
    alerter->raise(example_alert_foo);
    if(raises & 0b1) // if even
    {
        alerter->clear(example_alert_foo);
    }
    // Test acking stuff (if we need this in the future)
  //TODO implement ack tests
}
