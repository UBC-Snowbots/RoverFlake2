#include "rclcpp/rclcpp.hpp"


class WatchdogNode : public rclcpp::Node
{
public:
    WatchdogNode() : Node("Watchdog"){
               // Create a timer that fires every 500 milliseconds
    healthCheckTimer = this->create_wall_timer(
        std::chrono::milliseconds(1000),  // Timer interval
        std::bind(&WatchdogNode::cycleNodeHealth, this) // Callback function
    );
    }

    ~WatchdogNode(){
        RCLCPP_WARN(this->get_logger(), "WARNING: WATCHDOG NODE OFFLINE!");
    }

    void cycleNodeHealth();
private:

rclcpp::TimerBase::SharedPtr healthCheckTimer; // Timer handle


};


