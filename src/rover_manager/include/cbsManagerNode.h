// #pragma once
#include "cbsDevice.h"
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "rover_utils/include/fancyOutput.h"

using namespace ConsoleFormat;
class CBSManagerNode : public rclcpp::Node
{
public:
    CBSManagerNode() : Node("CBSManagerNode"){
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    arm_panel_publisher = this->create_publisher<rover_msgs::msg::ArmPanel>("/cbs/arm_panel", qos);
    // rclcpp::Parameter("expected_nodes", std::vector<std::string>({"Watchdog"}));
    // this->declare_parameter("expected_nodes", std::vector<std::string>({"Watchdog"}));
    // this->declare_parameter<std::vector<std::string>>("expected_nodes", {});

 
    // attachPort();

    // one.cbs_id.clear();
    // one.setID("one");
    // one = CBSDevice("", 9600, "meow");
    // devices.push_back(one);
    // one.setLogger(&this->get_logger());
    ArmPanel.initalize("/dev/ttyUSB0", 19200, "Arm Panel", this);
    ArmPanel.testPort("/dev/ttyUSB0", 19200);
    
   arm_panel_timer = this->create_wall_timer( //Timer setup if we need it
        std::chrono::milliseconds(10),  // Timer interval
        std::bind(&CBSManagerNode::armPanelPoll, this) // Callback function
    );


    }

    ~CBSManagerNode(){
        RCLCPP_WARN(this->get_logger(), "WARNING: CONTROL BASE MANAGER NODE OFFLINE!");
    }
    rclcpp::Publisher<rover_msgs::msg::ArmPanel>::SharedPtr arm_panel_publisher;
    void armPanelPoll();
private:

    rclcpp::TimerBase::SharedPtr arm_panel_timer; // Timer handle if we need it
    // bool attachPort(std::string port = "", int baudrate = 9600, int id = 0);
    std::vector<CBSDevice> devices;
  
  CBSDevice ArmPanel;
    // CBSDevice two;

};
