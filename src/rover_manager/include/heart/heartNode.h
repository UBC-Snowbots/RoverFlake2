#include <rclcpp/rclcpp.hpp>


class HeartNode : public rclcpp::Node
{
public:
    HeartNode() : Node("broken_heart")
    {    
        std::string name = this->get_name();
    
        if(name == "broken_heart"){
        RCLCPP_ERROR(this->get_logger(), "Sorry, you need to launch me with a different name. I'm going to get confused otherwise.");
            rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "I am %s", this->get_name());

    }
};