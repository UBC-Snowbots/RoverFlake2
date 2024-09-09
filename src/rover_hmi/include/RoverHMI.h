#pragma once 
#include <gtkmm.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rover_msgs/msg/arm_command.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <arm_hardware_interface/ArmSerialProtocol.h>

//opencv and image processing
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <sstream>
#include <iomanip>  // For std::setprecision




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
            arm_status_sub = this->create_subscription<rover_msgs::msg::ArmCommand>(
                "/arm/feedback", qos, 
                std::bind(&MainHMINode::armFeedbackCallback, this, std::placeholders::_1));
            arm_cmd_pub = this->create_publisher<rover_msgs::msg::ArmCommand>(
                "/arm/command", qos);

            image_feed_sub = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera1/image_raw", 10,
                std::bind(&MainHMINode::image_feed_callback, this, std::placeholders::_1));
            cmd_vel_monitor_sub = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", 10,
                std::bind(&MainHMINode::cmdVelCallback, this, std::placeholders::_1));
    
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
            builder->get_widget("middle_stack", middle_stack);\

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

            builder->get_widget("home_all_button", home_all_button);
                home_all_button->signal_clicked().connect(sigc::mem_fun(*this, &MainHMINode::handleHomeAllButtonClick));
            builder->get_widget("pos_feed_on_button", pos_feed_on_button);
                pos_feed_on_button->signal_clicked().connect(sigc::mem_fun(*this, &MainHMINode::handlePosFeedOnButtonClick));
            builder->get_widget("pos_feed_off_button", pos_feed_off_button);
                pos_feed_off_button->signal_clicked().connect(sigc::mem_fun(*this, &MainHMINode::handlePosFeedOffButtonClick));
            builder->get_widget("test_limits_button", test_limits_button);
                test_limits_button->signal_clicked().connect(sigc::mem_fun(*this, &MainHMINode::handleTestLimitsButtonClick));

            builder->get_widget("arm_abort_button", arm_abort_button);
                arm_abort_button->signal_clicked().connect(sigc::mem_fun(*this, &MainHMINode::handleArmAbortButtonClick));      
            builder->get_widget("a1_readout_pos", axis_pos_label[0]);
            builder->get_widget("a2_readout_pos", axis_pos_label[1]);
            builder->get_widget("a3_readout_pos", axis_pos_label[2]);
            builder->get_widget("a4_readout_pos", axis_pos_label[3]);
            builder->get_widget("a5_readout_pos", axis_pos_label[4]);
            builder->get_widget("a6_readout_pos", axis_pos_label[5]);

            builder->get_widget("inc_axis_1_button", inc_axis_button[0]);
            builder->get_widget("inc_axis_2_button", inc_axis_button[1]);
            builder->get_widget("inc_axis_3_button", inc_axis_button[2]);
            builder->get_widget("inc_axis_4_button", inc_axis_button[3]);
            builder->get_widget("inc_axis_5_button", inc_axis_button[4]);
            builder->get_widget("inc_axis_6_button", inc_axis_button[5]);
            

            builder->get_widget("dec_axis_1_button", dec_axis_button[0]);
            builder->get_widget("dec_axis_2_button", dec_axis_button[1]);
            builder->get_widget("dec_axis_3_button", dec_axis_button[2]);
            builder->get_widget("dec_axis_4_button", dec_axis_button[3]);
            builder->get_widget("dec_axis_5_button", dec_axis_button[4]);
            builder->get_widget("dec_axis_6_button", dec_axis_button[5]);

            for(int i = 0; i < 6; i++){
                inc_axis_button[i]->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleIncAxisButtonClick), i));
                inc_axis_button[i]->signal_released().connect(sigc::mem_fun(*this, &MainHMINode::handleAxisButtonRelease));
                dec_axis_button[i]->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleDecAxisButtonClick), i));
                dec_axis_button[i]->signal_released().connect(sigc::mem_fun(*this, &MainHMINode::handleAxisButtonRelease));

            }




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
    std::string floatToStringTruncate(float value, int decimals);

    bool handleVideoFrameDraw(const Cairo::RefPtr<Cairo::Context>& cr);
    void image_feed_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    private:
    std::string main_css_file_path;
    void armFeedbackCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg);
    // rclcpp::TimerBase
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void handleHomeAllButtonClick();
    void handlePosFeedOnButtonClick();
    void handlePosFeedOffButtonClick();
    void handleIncAxisButtonClick(int index); //RELATIVE VELOCITIERSs
    void handleDecAxisButtonClick(int index);
    void handleAxisButtonRelease();
    void handleArmAbortButtonClick();
    void handleTestLimitsButtonClick();
    
    // void armFeebackCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg);

    Gtk::Window* middle_window;
    Gtk::Stack* middle_stack; //Like a deck of cards, each card is a different screen we can view

    Gtk::Widget* image_draw_area;

    Gtk::Label* axis_pos_label[6];

    Gtk::Button* home_all_button;
    Gtk::Button* pos_feed_on_button;
    Gtk::Button* pos_feed_off_button;
    Gtk::Button* test_limits_button;


    Gtk::Button* arm_abort_button; //! Arm Abort Button

    Gtk::Button* dec_axis_button[6];
    Gtk::Button* inc_axis_button[6];
  


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
    
    rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr arm_status_sub;
    rclcpp::Publisher<rover_msgs::msg::ArmCommand>::SharedPtr arm_cmd_pub;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_feed_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_monitor_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr six_motor_monitor_sub;
    
    cv::Mat image_;
    Glib::RefPtr<Gdk::Pixbuf> pixbuf_;
    std::mutex image_mutex_;
};


/* Topics to sub to

    - Joint states. if we want arm stuff, or pure rviz?
    - Enviroment data
    - BMS data







 */