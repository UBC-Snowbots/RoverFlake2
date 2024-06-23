#pragma once 
#include <gtkmm.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rover_msgs/msg/arm_command.h>

#include <chrono>
#include <ctime>
#include <memory>

#define QUEUE_SIZE 20

class MainHMINode : public rclcpp::Node, Gtk::Window
{
    public: 
        MainHMINode() : Node("main_hmi_node") {
            set_title("middle");
            std::string glade_file_path = this->package_share_dir + "/glade_files/left_screen.glade";

            auto builder = Gtk::Builder::create_from_file(glade_file_path.c_str());
            auto qos = rclcpp::QoS(rclcpp::KeepLast(QUEUE_SIZE)).transient_local;
            arm_status_subscriber = this->create_subscription<rover_msgs::msg::ArmCommand>("/arm/feedback", qos, std::bind(&MainHMINode::armFeebackCallback, this, std::placeholders::_1));
        }


    void run(){
        app->run(*middle_window);
    }

    std::string css_file_path;    
    private:
    
    void armFeedbackCallback(const rover_msgs::msg::ArmCommand::SharedPtr& msg);
    // rclcpp::TimerBase

    void load_css(const Glib::RefPtr<Gtk::CssProvider>& css_provider);
    
    void armFeebackCallback(const rover_msgs::msg::ArmStatus::SharedPtr msg);

    Gtk::Window* middle_window;
    std::string package_share_dir;
    
    rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr arm_status_subscriber;
};

void MainHMINode::armFeedbackCallback(const rover_msgs::msg::ArmCommand::SharedPtr& msg){
    RCLCPP_INFO(this->get_logger(), "meow arm");
}