#pragma once 
#include <gtkmm.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rover_msgs/msg/arm_command.hpp>

//opencv and image processing
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>


//standard c++ stuff, some may alreaddy be included in rclcpp
#include <chrono>
#include <ctime>
#include <memory>
#include <thread>
// #include <signal.h>

#define QUEUE_SIZE 20
#define RCL_SPIN_RATE 30

class MainHMINode : public rclcpp::Node, public Gtk::Window
{
    public: 
        MainHMINode() : Node("main_hmi_node") {
            set_title("middle");
            
            this->package_share_dir = ament_index_cpp::get_package_share_directory("rover_hmi");

            std::string glade_file_path = this->package_share_dir + "/glade_files/middle_screen.glade";
            RCLCPP_INFO(this->get_logger(), glade_file_path.c_str());
            auto builder = Gtk::Builder::create_from_file(glade_file_path.c_str());
            auto qos = rclcpp::QoS(rclcpp::KeepLast(QUEUE_SIZE)).transient_local();
            arm_status_subscriber = this->create_subscription<rover_msgs::msg::ArmCommand>(
                "/arm/feedback", qos, 
                std::bind(&MainHMINode::armFeedbackCallback, this, std::placeholders::_1));

            image_feed_subscription = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera1/image_raw", 10,
                std::bind(&MainHMINode::image_feed_callback, this, std::placeholders::_1));
    
            //* css files
            main_css_file_path = this->package_share_dir + "/css_files/main_style.css";
            RCLCPP_INFO(this->get_logger(), main_css_file_path.c_str());
            auto css_provider = Gtk::CssProvider::create();
            load_css(css_provider);
            RCLCPP_INFO(this->get_logger(), "Meowing css");
            
            auto screen = Gdk::Screen::get_default();
            auto style_context = get_style_context();
            style_context->add_provider_for_screen(screen, css_provider, GTK_STYLE_PROVIDER_PRIORITY_USER);
            RCLCPP_INFO(this->get_logger(), "Meowing builder");
         

            //* Setup GTK widgets
            builder->get_widget("middle_window", middle_window);
            builder->get_widget("middle_stack", middle_stack);

            //*build the system overview card
            builder->get_widget("subsystem_status_grid", subsys_grid.grid);
            subsys_grid.grid->signal_draw().connect(sigc::mem_fun(*this, &MainHMINode::handleSubsystemStatusGridDraw));
                builder->get_widget("comms_online_status_label", subsys_grid.comms_online_status_label);
                builder->get_widget("arm_online_status_label", subsys_grid.arm_online_status_label);
                builder->get_widget("drive_online_status_label", subsys_grid.drive_online_status_label);
                builder->get_widget("ptz_online_status_label", subsys_grid.ptz_online_status_label);
                builder->get_widget("lights_online_status_label", subsys_grid.lights_online_status_label);

                builder->get_widget("comms_misc_status_label", subsys_grid.comms_misc_status_label);
                builder->get_widget("arm_misc_status_label", subsys_grid.arm_misc_status_label);
                builder->get_widget("drive_misc_status_label", subsys_grid.drive_misc_status_label);
                builder->get_widget("ptz_misc_status_label", subsys_grid.ptz_misc_status_label);
                builder->get_widget("lights_misc_status_label", subsys_grid.lights_misc_status_label);
            // builder->get_widget("navig_misc_status_label", ptz_misc_status_label);
            
            builder->get_widget("image_draw_area", image_draw_area);
                image_draw_area->signal_draw().connect(sigc::mem_fun(*this, &MainHMINode::handleVideoFrameDraw));
            // changeCard("full_control_card");
            RCLCPP_INFO(this->get_logger(), "Meowing complete");

        }


    void run(){
            RCLCPP_INFO(this->get_logger(), "Meowing start");

        app->run(*middle_window);
            RCLCPP_INFO(this->get_logger(), "Meowing run");

    }
    std::string current_middle_card = "system_overview_card";
    void changeCard(std::string target_card);


    Glib::RefPtr<Gtk::Application> app;
    void load_css(const Glib::RefPtr<Gtk::CssProvider>& css_provider);
    //*Draw functions, can redraw widgets based on this node's data
    bool handleSubsystemStatusGridDraw(const Cairo::RefPtr<Cairo::Context>& context);
    
    bool handleVideoFrameDraw(const Cairo::RefPtr<Cairo::Context>& cr);
    void image_feed_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    private:
    std::string main_css_file_path;
    void armFeedbackCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg);
    // rclcpp::TimerBase

    
    // void armFeebackCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg);

    Gtk::Window* middle_window;
    Gtk::Stack* middle_stack; //Like a deck of cards, each card is a different screen we can view

    Gtk::Widget* image_draw_area;

    //* System Overview
    struct SubsystemGrid{
        Gtk::Grid* grid;
            Gtk::Label* comms_online_status_label; 
                Gtk::Label* comms_misc_status_label; 
                std::string comms_online_status_string = "NULL ERR";
                std::string comms_misc_status_string = "NULL ERR";
         
            Gtk::Label* arm_online_status_label; 
                Gtk::Label* arm_misc_status_label;
                std::string arm_online_status_string = "NULL ERR";
                std::string arm_misc_status_string = "NULL ERR";
         
            Gtk::Label* drive_online_status_label; 
                Gtk::Label* drive_misc_status_label;
                std::string drive_online_status_string = "NULL ERR";
                std::string drive_misc_status_string = "NULL ERR";
         
            Gtk::Label* ptz_online_status_label; 
                Gtk::Label* ptz_misc_status_label;
                std::string ptz_online_status_string = "NULL ERR"; 
                std::string ptz_misc_status_string = "NULL ERR";

            Gtk::Label* navigation_online_status_label; 
                Gtk::Label* navigation_misc_status_label;
                std::string navigation_online_status_string = "NULL ERR";
                std::string navigation_misc_status_string = "NULL ERR";

            Gtk::Label* lights_online_status_label; 
                Gtk::Label* lights_misc_status_label;
                std::string lights_online_status_string = "NULL ERR";
                std::string lights_misc_status_string = "NULL ERR";
    };

    SubsystemGrid subsys_grid;


    









    std::string package_share_dir;
    
    rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr arm_status_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_feed_subscription;
    cv::Mat image_;
    Glib::RefPtr<Gdk::Pixbuf> pixbuf_;
    std::mutex image_mutex_;
};


/* Topics to sub to

    - Joint states. if we want arm stuff, or pure rviz?
    - Enviroment data
    - BMS data







 */