#pragma once
// #include "cbsManagerNode.h" //FUCKOUTTAHERE
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "cbsDefinitions.h"
#include "rover_msgs/msg/arm_panel.hpp"
#include "rover_msgs/msg/generic_panel.hpp"
class CBSHardwareManagerNode; //FORWARD DECLARRATIONS

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
    void init(int baudrate_, std::string new_id, PARSE_SEQUENCE parse_seq_, CBSHardwareManagerNode* manager_);
    // void setID(std::string my_id);
    // void setLogger(rclcpp::Logger* main_logger);

    int testPort(std::string port_path, int baudrate);
    int findMyPort();
    void pollRX();
    void setMinMsgSize(int new_size);

    bool AreYouSureImPluggedIn = true;
    int failed_connection_attempts = 0;
    std::string id = "";
    PARSE_SEQUENCE parse_seq;
private:
    serial::Serial serial;
    // rclcpp::Logger& logger_;
    // rclcpp::TimerBase::SharedPtr healthCheckTimer; // Timer handle if we need it
    CBSHardwareManagerNode* manager;
    bool attachPort(std::string port_path = "", int baudrate = 9600, int id = 0);
    void parseArmJoyPanelBuff(std::string buff);
    void parseLeftPanelABuff(std::string buff);
    void parseGenericBuff(std::string buff);
    // bool openPort();


    std::string port_path = "";
    int baudrate = 9600;
    int min_msg_size = 4;
    bool port_found = 0;
    bool ready_for_polling = 0;
    
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
