//* Meant to be like, the heart of the rover, each subsystem can be thought of as a muscle or organ. the heart can start/shutoff the flow of blood to each subsystem
//* One of these will run on each computer, and run/manage all nodes on that device

#include <rclcpp/rclcpp.hpp>
#include <rover_utils/include/roverCommon.h>
#include <rover_msgs/msg/heart_request.hpp>
using namespace ConsoleFormat;
class HeartNode : public rclcpp::Node
{
public:
    HeartNode() : Node("broken_heart", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
    {    
        std::string my_name = this->get_name();
    
        if(my_name == "broken_heart"){
        RCLCPP_ERROR(this->get_logger(), "Sorry, you need to launch me with a different name (use a launch file with name= ). I'm going to get confused otherwise.");
            rclcpp::shutdown();
    }
    std::string request_topic = my_name + "/request";
    // std::string request_topic = my_name;
    
    this->get_parameters("subsystems", params);
    // this->get_parameter("request_topic", request_topic);
    
    
    RCLCPP_INFO(this->get_logger(), "I am %s, on topic %s", this->get_name(), request_topic.c_str());

    //* Construct Subsystems from Params
    { //just making a little scope for i, sure theres a better way
    int i = 0;
    for (const auto& [key, param] : params) {
        RCLCPP_INFO(this->get_logger(), "Got Subystem %s%s%s = %s%s%s", 
                    green(), key.c_str(), reset(), bright_blue(), param.value_to_string().c_str(), reset());
            SubSystem new_subsys;
            new_subsys.name = key;
            new_subsys.exec_command = param.value_to_string();
            new_subsys.index = i;
            subsystems_vector.push_back(new_subsys);
            subsystems[key] = new_subsys;
            i++;
    }
    if(i == 0){
        RCLCPP_WARN(this->get_logger(), "Warning, no subsystems defined. You sure you loaded parameters right?");
    }
    }
    heart_request_sub = this->create_subscription<rover_msgs::msg::HeartRequest>(
        request_topic, 10, std::bind(&HeartNode::heartRequestCallback, this, std::placeholders::_1));

    // runSubSystem(subsystems_vector[0]);
    }
private: 
struct SubSystem{
    std::string name;
    std::string exec_command;
    int index = -1;
    pid_t pid;
    pid_t gpid;
    pid_t sid;
    bool online = false;
};
std::map<std::string, rclcpp::Parameter> params;
std::unordered_map<std::string, SubSystem> subsystems;

// std::unordered_map<std::string, std::SubSystem>;


std::vector<SubSystem> subsystems_vector;

void runSubSystem(SubSystem& subsys);
void killSubSystem(SubSystem& subsys);

// void runChildNode(std::string pkg, std::string node_or_launch_file, std::string subsytem_name, int type = NODE, bool kill_orphan = true);
void killProcessGroup(pid_t pgid); //! Should prob have a error return



    rclcpp::Subscription<rover_msgs::msg::HeartRequest>::SharedPtr heart_request_sub;
        void heartRequestCallback(const rover_msgs::msg::HeartRequest::SharedPtr request);
};




