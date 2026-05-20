#include <rclcpp/rclcpp.hpp>
// #include <rover_utils/include/alertRaiser.h>
#include "../../include/alertRaiser.h"


class AlertExampleNode : public rclcpp::Node 
{
    public:
    AlertExampleNode();

    private:
    // AlertRaiser* alerter;
    std::unique_ptr<AlertRaiser> alerter;
    // Alerts
    // Easiest to initialize an arrayed alert from the constructor
    Alert example_alert_foo = {};
    Alert example_alert_bar = {};
    Alert example_arrayed_alert[500] = {};  // 


    // AlertRaiser alerter;
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer;


};