#include "rclcpp/rclcpp.hpp"


class WatchdogNode : public rclcpp::Node
{
public:
    WatchdogNode() : Node("Watchdog"){

    // rclcpp::Parameter("expected_nodes", std::vector<std::string>({"Watchdog"}));
    this->declare_parameter("expected_nodes", std::vector<std::string>({"Watchdog"}));
    // this->declare_parameter<std::vector<std::string>>("expected_nodes", {});

    // healthCheckTimer = this->create_wall_timer(
    //     std::chrono::milliseconds(1000),  // Timer interval
    //     std::bind(&WatchdogNode::cycleNodeHealth, this) // Callback function
    // );





    }

    ~WatchdogNode(){
        RCLCPP_WARN(this->get_logger(), "WARNING: WATCHDOG NODE OFFLINE!");
    }

    void cycleNodeHealth();
private:

    rclcpp::TimerBase::SharedPtr healthCheckTimer; // Timer handle



};


