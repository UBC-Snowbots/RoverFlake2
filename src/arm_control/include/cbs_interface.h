#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/arm_panel.hpp"
#include "rover_msgs/msg/arm_command.hpp"
#include "armControlParams.h"



class CBSArmInterface : public rclcpp::Node
{
public:
    CBSArmInterface() : Node("CBSArmInterface"){
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    arm_cmd_publisher = this->create_publisher<rover_msgs::msg::ArmCommand>("/arm/command", qos);

     arm_panel_subscriber = this->create_subscription<rover_msgs::msg::ArmPanel>(
            "/cbs/arm_panel", 10, std::bind(&CBSArmInterface::arm_panel_callback, this, std::placeholders::_1));
 
    
//    arm_panel_timer = this->create_wall_timer( //Timer setup if we need it
//         std::chrono::milliseconds(10),  // Timer interval
//         std::bind(&CBSManagerNode::armPanelPoll, this) // Callback function
//     );


    }

    ~CBSArmInterface(){
        RCLCPP_WARN(this->get_logger(), "WARNING: ARM PANEL INTERFACE NODE OFFLINE!");
    }
    rclcpp::Publisher<rover_msgs::msg::ArmCommand>::SharedPtr arm_cmd_publisher;
    rclcpp::Subscription<rover_msgs::msg::ArmPanel>::SharedPtr arm_panel_subscriber;
    // rclcpp::Publisher<
private:
    void arm_panel_callback(const rover_msgs::msg::ArmPanel::SharedPtr msg);
    float max_speed_deg[NUM_JOINTS] = {80, 40, 80, 80, 80, 80};

    // rclcpp::TimerBase::SharedPtr arm_panel_timer; // Timer handle if we need it


};
