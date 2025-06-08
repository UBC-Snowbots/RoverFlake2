//Quick node to route commands from cbs panels

// #include "sample_node.h"


#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/generic_panel.hpp"


class RouterA : public rclcpp::Node {
public:
    RouterA() : Node("router_a") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        //command_publisher_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", qos);
        //gear_publisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", qos);

        // timer_ = this->create_wall_timer(
        // std::chrono::duration<double>(period),std::bind(&ManualControlNode::test_send, this));
        left_panel_a_sub = this->create_subscription<rover_msgs::msg::GenericPanel>(
            "/cbs/left_panel_a", 10, std::bind(&RouterA::leftPanelACallback, this, std::placeholders::_1));
    }



    

    // void test_send(){
    //     send_command(0.5, 1.0, 1.0, 0.5);
    //     // rclcpp::logger

    // }

private:
 
    // rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr command_publisher_;
   
    void leftPanelACallback(const rover_msgs::msg::GenericPanel::SharedPtr msg);

    rclcpp::Subscription<rover_msgs::msg::GenericPanel>::SharedPtr left_panel_a_sub;

    rclcpp::TimerBase::SharedPtr timer_;

};

void RouterA::leftPanelACallback(const rover_msgs::msg::GenericPanel::SharedPtr msg){
    //decode and do whatnot here. This is where we decide what the buttons do, for the most part.
    //We can also just use callbacks within certain nodes
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RouterA>();

    // Example: send a command with a steering angle of 0.5 rad and speed of 1.0 m/s

    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}