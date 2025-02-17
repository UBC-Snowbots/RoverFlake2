#pragma once
// #include "cbsManagerNode.h" //FUCKOUTTAHERE
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "cbsDefinitions.h"
#include "rover_msgs/msg/arm_panel.hpp"
class CBSManagerNode; //FORWARD DECLARRATIONS

class CBSDevice
{
public:
    // CBSDevice(std::string port_path, int baudrate, std::string cbs_id = "generic_unset") : port_path(port_path), baudrate(baudrate), cbs_id(cbs_id) { 
    CBSDevice() : manager(nullptr){
    // rclcpp::Parameter("expected_nodes", std::vector<std::string>({"Watchdog"}));
    // this->declare_parameter("expected_nodes", std::vector<std::string>({"Watchdog"}));
    // this->declare_parameter<std::vector<std::string>>("expected_nodes", {});

    // healthCheckTimer = this->create_wall_timer( //Timer setup if we need it
    //     std::chrono::milliseconds(1000),  // Timer interval
    //     std::bind(&WatchdogNode::cycleNodeHealth, this) // Callback function
    // );
    // attachPort();





    }

    ~CBSDevice(){
        // RCLCPP_WARN(logger_, "WARNING: CONTROL BASE DEVICE: %s NODE OFFLINE!", cbs_id.c_str());
    }
    
    // void setPortPath(std::string port_path);
    void initalize(std::string port_path, int baudrate, std::string id, CBSManagerNode* manager_);
    void setID(std::string id);
    // void setLogger(rclcpp::Logger* main_logger);

    serial::Serial serial;
    int testPort(std::string port_path, int baudrate);
    void pollRX();
private:
    // rclcpp::Logger& logger_;
    // rclcpp::TimerBase::SharedPtr healthCheckTimer; // Timer handle if we need it
    CBSManagerNode* manager;
    bool attachPort(std::string port_path = "", int baudrate = 9600, int id = 0);
    void parseBuff(std::string buff);
    // bool openPort();


    std::string port_path = "";
    int baudrate = 9600;
    std::string cbs_id = "";
    int min_msg_size = 4;
    
    // int id;
    // struct potentiometer{
    //     int i = -1;
    //     float current_val;
    // }
    //* POTENTIOMETERS
    int num_pots = -1;
    float current_pot_vals;

    //*BUTTONS - Might need some more logic as they are momentary 
    int num_buttons = -1;
    bool current_butt_vals;

    //*SWITCHES
    int num_switches = -1;
    int current_switch_vals; //* 2way switches are 0, 1. 3way switches are -1, 0, 1

};
