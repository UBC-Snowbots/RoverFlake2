#include "rclcpp/rclcpp.hpp"


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
    attachPort();
    





    }

    ~CBSManagerNode(){
        RCLCPP_WARN(this->get_logger(), "WARNING: CONTROL BASE MANAGER NODE OFFLINE!");
    }

    void cycleNodeHealth();
private:

    // rclcpp::TimerBase::SharedPtr healthCheckTimer; // Timer handle if we need it
    bool attachPort(std::string port = "", int baudrate = 9600, int id = 0);


};
