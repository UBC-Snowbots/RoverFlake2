// #pragma once
#include "cbsDevice.h"
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "rover_utils/include/fancyOutput.h"

// using ConsoleFormat;
class CBSManagerNode : public rclcpp::Node
{
public:
    CBSManagerNode() : Node("CBSManagerNode"){

    // rclcpp::Parameter("expected_nodes", std::vector<std::string>({"Watchdog"}));
    // this->declare_parameter("expected_nodes", std::vector<std::string>({"Watchdog"}));
    // this->declare_parameter<std::vector<std::string>>("expected_nodes", {});

    // healthCheckTimer = this->create_wall_timer( //Timer setup if we need it
    //     std::chrono::milliseconds(1000),  // Timer interval
    //     std::bind(&WatchdogNode::cycleNodeHealth, this) // Callback function
    // );
    // attachPort();

    // one.cbs_id.clear();
    // one.setID("one");
    // one = CBSDevice("", 9600, "meow");
    // devices.push_back(one);
    // one.setLogger(&this->get_logger());
    ArmPanel.initalize("/dev/USBtty0", 19200, "Arm Panel", this);
    ArmPanel.testPort("/dev/USBtty0", 19200);
    



    }

    ~CBSManagerNode(){
        RCLCPP_WARN(this->get_logger(), "WARNING: CONTROL BASE MANAGER NODE OFFLINE!");
    }

private:

    // rclcpp::TimerBase::SharedPtr healthCheckTimer; // Timer handle if we need it
    // bool attachPort(std::string port = "", int baudrate = 9600, int id = 0);
    std::vector<CBSDevice> devices;
  
  CBSDevice ArmPanel;
    // CBSDevice two;

};
