#pragma once 
#include <gtkmm.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rover_msgs/msg/arm_command.hpp>

#include <chrono>
#include <ctime>
#include <memory>
#include <thread>

#define QUEUE_SIZE 20

class MainHMINode : public rclcpp::Node, Gtk::Window
{
    public: 
        MainHMINode() : Node("main_hmi_node") {
            set_title("middle");
            
            this->package_share_dir = ament_index_cpp::get_package_share_directory("rover_hmi");

            std::string glade_file_path = this->package_share_dir + "/glade_files/left_screen.glade";
            RCLCPP_INFO(this->get_logger(), glade_file_path.c_str());
            auto builder = Gtk::Builder::create_from_file(glade_file_path.c_str());
            auto qos = rclcpp::QoS(rclcpp::KeepLast(QUEUE_SIZE)).transient_local();
            arm_status_subscriber = this->create_subscription<rover_msgs::msg::ArmCommand>("/arm/feedback", qos, std::bind(&MainHMINode::armFeedbackCallback, this, std::placeholders::_1));

    
            //* css files
            main_css_file_path = this->package_share_dir + "/css_files/main_style.css";
            RCLCPP_INFO(this->get_logger(), main_css_file_path.c_str());
            auto css_provider = Gtk::CssProvider::create();
            load_css(css_provider);
            auto screen = Gdk::Screen::get_default();
            auto style_context = get_style_context();
            style_context->add_provider_for_screen(screen, css_provider, GTK_STYLE_PROVIDER_PRIORITY_USER);
         

            //* Setup GTK widgets
            builder->get_widget("middle_window", middle_window);
            builder->get_widget("middle_stack", middle_stack);

            changeCard("full_control_card");

        }


    void run(){
        app->run(*middle_window);
    }
    std::string current_middle_card = "system_overview_card";
    void changeCard(std::string target_card);


    Glib::RefPtr<Gtk::Application> app;
    void load_css(const Glib::RefPtr<Gtk::CssProvider>& css_provider);

    private:
    std::string main_css_file_path;
    void armFeedbackCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg);
    // rclcpp::TimerBase

    
    // void armFeebackCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg);

    Gtk::Window* middle_window;
    Gtk::Stack* middle_stack; //Like a deck of cards, each card is a different screen we can view
    std::string package_share_dir;
    
    rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr arm_status_subscriber;
};

