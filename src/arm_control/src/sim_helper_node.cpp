#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/arm_command.hpp" // Include your custom message
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class ArmCommandNode : public rclcpp::Node
{
public:
    ArmCommandNode() : Node("arm_command_node")
    {
        // Subscriber to /arm/command
        arm_command_subscriber_ = this->create_subscription<rover_msgs::msg::ArmCommand>(
            "/arm/command", 10,
            std::bind(&ArmCommandNode::armCommandCallback, this, std::placeholders::_1));

        // Publisher to /arm/sim_command
        sim_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/arm/sim_command", 10);
        
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);
        
    }

private:
    void armCommandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg)
    {
        // Prepare the Float64MultiArray message
        auto sim_command_msg = std_msgs::msg::Float64MultiArray();
        sim_command_msg.data.resize(6, 0.0); // Resize to 6 and initialize to 0

        // Copy the velocities from the incoming ArmCommand message
        for (size_t i = 0; i < std::min(msg->velocities.size(), static_cast<size_t>(6)); ++i)
        {
            sim_command_msg.data[i] = msg->velocities[i];
        }

        // Publish the message
        sim_command_publisher_->publish(sim_command_msg);
    }

    void armFeedbackCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
        // sensor_msgs::msg::JointState joint_states_msg = 
    }

    rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr arm_command_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr sim_command_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmCommandNode>());
    rclcpp::shutdown();
    return 0;
}
