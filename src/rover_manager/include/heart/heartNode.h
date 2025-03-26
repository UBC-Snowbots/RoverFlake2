#include <rclcpp/rclcpp.hpp>
#include <rover_utils/include/roverCommon.h>
using namespace ConsoleFormat;
class HeartNode : public rclcpp::Node
{
public:
    HeartNode() : Node("broken_heart", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
    {    
        std::string name = this->get_name();
    
        if(name == "broken_heart"){
        RCLCPP_ERROR(this->get_logger(), "Sorry, you need to launch me with a different name (use a launch file with name= ). I'm going to get confused otherwise.");
            rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "I am %s", this->get_name());

    this->get_parameters("subsystems", params);
    // auto subsystems = this->get_node_parameters_interface()
    // ->get_parameters_by_prefix("subsystems");
    //* Construct Subsystems
    for (const auto& [key, param] : params) {
        RCLCPP_INFO(this->get_logger(), "Got Subystem %s%s%s = %s%s%s", 
                    green(), key.c_str(), reset(), bright_blue(), param.value_to_string().c_str(), reset());
            SubSystem new_subsys
            subsystems.push_back()
    }

    }
private: 
std::map<std::string, rclcpp::Parameter> params;
std::vector<SubSystem> subsystems;
// std::unordered_map<std::string, std::SubSystem>;

struct SubSystem{
    std::string name;
    std::string exec_command;
    pid_t pid;
    pid_t gpid;
    pid_t sid;
    bool online = false;
};


void runSubSystem(SubSystem& subsys);

    // rclcpp::Subscription
};


void HeartNode::runSubSystem(SubSystem& subsys){

}