// #pragma once

/*
This node manages and interfaces with the hardware of the control base. 
    It will open up TTY ports, and find witch device is witch based on just the output of the device.
    With this, we can create multiple devices, with or without differing baudrates, and not worry about wich usb port they are plugged into.
    Setting the serial ids would be way easier, but not all arduinos/MCUs have this function.
    ! Currently, will not be able to open ACM ports, but that is an easy fix.
*/
#include "cbsDevice.h"
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "rover_utils/include/fancyOutput.h"
using namespace ConsoleFormat;
class CBSHardwareManagerNode : public rclcpp::Node
{
public:
    CBSHardwareManagerNode() : Node("CBSHardwareManagerNode"){
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    arm_panel_publisher = this->create_publisher<rover_msgs::msg::ArmPanel>(TOPIC_ARM_PANEL, qos);
    left_panel_A_publisher = this->create_publisher<rover_msgs::msg::GenericPanel>(TOPIC_LEFT_PANEL_A, qos);
    
    // rclcpp::Parameter("expected_nodes", std::vector<std::string>({"Watchdog"}));
    // this->declare_parameter("expected_nodes", std::vector<std::string>({"Watchdog"}));
    // this->declare_parameter<std::vector<std::string>>("expected_nodes", {});
    std::system("notify-send 'CBS hardware manager STARTING UP'");
    
    // attachPort();

    // one.cbs_id.clear();
    // one.setID("one");
    // one = CBSDevice("", 9600, "meow");
    // devices.push_back(one);
    // one.setLogger(&this->get_logger());
    //TODO needs cleaning up but she works
    ArmJoyPanel.init(19200, "arm_joy_panel", PARSE_SEQUENCE::ARM_JOY_PANEL, this);
    LeftPanel_A.init(9600,"gen_left_A", PARSE_SEQUENCE::LEFT_PANEL_A, this);

    connectDevice(ArmJoyPanel);
    connectDevice(LeftPanel_A);
    // while(ArmJoyPanel.findMyPort() != PORT_FOUND_SUCCESS){
    //     //wait until panel finds it's port. Sometimes it needs to go through all ports multiple times
    //     // break;
    // }
    // rclcpp::sleep_for(std::chrono::seconds(1));
    // while(LeftPanel_A.findMyPort() != PORT_FOUND_SUCCESS && LeftPanel_A.AreYouSureImPluggedIn){
    //     //wait until panel finds it's port. Sometimes it needs to go through all ports multiple times
    //     LeftPanel_A.failed_connection_attempts++;
    //     if(LeftPanel_A.failed_connection_attempts > FAILED_CONNECTION_ATTEMPTS_MAX){
    //         LeftPanel_A.AreYouSureImPluggedIn = false;
    //         RCLCPP_WARN(this->get_logger(), "Are you sure %s %s %s is plugged in dawg?", bold(), LeftPanel_A.id.c_str(), reset());
    //         rclcpp::sleep_for(std::chrono::seconds(5));
    //     }
    // }
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(this->get_logger(), "All ports found! Starting polling timers");
    std::system("notify-send 'CBS hardware manager READY'");

    ArmJoyPanel.setMinMsgSize(32);
    LeftPanel_A.setMinMsgSize(34);
    if(ArmJoyPanel.is_connected){
          arm_panel_timer = this->create_wall_timer( //Timer setup if we need it
        std::chrono::milliseconds(10),  // Timer interval
        std::bind(&CBSHardwareManagerNode::armPanelPoll, this) // Callback function
    );
    }
 //? don't really know when to check connections. following is checked during the timer
    slow_poller = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&CBSHardwareManagerNode::slowPollCycle, this)
    );
    

    }

    ~CBSHardwareManagerNode(){
        RCLCPP_WARN(this->get_logger(), "WARNING: CONTROL BASE MANAGER NODE OFFLINE!");
        std::system("notify-send 'CBS hardware manager OFFLINE'"); // sends a notification to the system. Helpful when we start it within a launch file and the terminal output is too crowded

    }
    rclcpp::Publisher<rover_msgs::msg::ArmPanel>::SharedPtr arm_panel_publisher;
    rclcpp::Publisher<rover_msgs::msg::GenericPanel>::SharedPtr left_panel_A_publisher;

    void armPanelPoll();
    void slowPollCycle();
    int connectDevice(CBSDevice& dev);

    std::vector<std::string> possible_ports = {"test", "test2","/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3", "/dev/ttyUSB4", "/dev/ttyUSB5", "/dev/ttyUSB6", "/dev/ttyUSB7"}; //TODO make this suck less
    std::vector<std::string> taken_ports;


private:

    rclcpp::TimerBase::SharedPtr arm_panel_timer; // Timer handle if we need it
    rclcpp::TimerBase::SharedPtr slow_poller;
    // bool attachPort(std::string port = "", int baudrate = 9600, int id = 0);
    std::vector<CBSDevice> devices;
  
  CBSDevice ArmJoyPanel;
  CBSDevice LeftPanel_A;
    // CBSDevice two;


};
