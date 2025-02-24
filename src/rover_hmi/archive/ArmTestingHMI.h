#include "RoverHMI.h"
#include <arm_control/include/armControlParams.h>


class ArmHMINode : public rclcpp::Node, public Gtk::Window
{
    public: 
        ArmHMINode() : Node("arm_hmi_node") {
            set_title("Arm Testing Panel | UBC ROVER");
            
            this->package_share_dir = ament_index_cpp::get_package_share_directory("rover_hmi");

            std::string glade_file_path = this->package_share_dir + "/glade_files/arm_testing_panel.glade"; //Grab the .glade file, made with the drag n drop editor Glade
            RCLCPP_INFO(this->get_logger(), glade_file_path.c_str());
            auto builder = Gtk::Builder::create_from_file(glade_file_path.c_str());
            auto qos = rclcpp::QoS(rclcpp::KeepLast(QUEUE_SIZE)).transient_local(); // Set Quality of Service to a simple queue

            arm_status_sub = this->create_subscription<rover_msgs::msg::ArmCommand>(
                "/arm/feedback", qos, 
                std::bind(&ArmHMINode::armFeedbackCallback, this, std::placeholders::_1));
            arm_cmd_pub = this->create_publisher<rover_msgs::msg::ArmCommand>(
                "/arm/command", qos);
    
            //* css files
            arm_css_file_path = this->package_share_dir + "/css_files/main_style.css";
            RCLCPP_INFO(this->get_logger(), arm_css_file_path.c_str());
            auto css_provider = Gtk::CssProvider::create();
            load_css(css_provider, arm_css_file_path);
            RCLCPP_INFO(this->get_logger(), "Meowing css");
            
            auto screen = Gdk::Screen::get_default();
            auto style_context = get_style_context();
            style_context->add_provider_for_screen(screen, css_provider, GTK_STYLE_PROVIDER_PRIORITY_USER);
            RCLCPP_INFO(this->get_logger(), "Meowing builder");
         

            //* Setup GTK widgets
            builder->get_widget("arm_hmi_window", arm_hmi_window);

            //*build the system overview card
                // builder->get_widget("subsystem_status_grid", subsys_grid.grid);
                // subsys_grid.grid->signal_draw().connect(sigc::mem_fun(*this, &MainHMINode::handleSubsystemStatusGridDraw));
                //     builder->get_widget("comms_online_status_label", subsys_grid.comms_online_status_label);
                //     builder->get_widget("arm_online_status_label", subsys_grid.arm_online_status_label);
                //     builder->get_widget("drive_online_status_label", subsys_grid.drive_online_status_label);
                //     builder->get_widget("ptz_online_status_label", subsys_grid.ptz_online_status_label);
                //     builder->get_widget("lights_online_status_label", subsys_grid.lights_online_status_label);

                //     builder->get_widget("comms_misc_status_label", subsys_grid.comms_misc_status_label);
                //     builder->get_widget("arm_misc_status_label", subsys_grid.arm_misc_status_label);
                //     builder->get_widget("drive_misc_status_label", subsys_grid.drive_misc_status_label);
                //     builder->get_widget("ptz_misc_status_label", subsys_grid.ptz_misc_status_label);
                //     builder->get_widget("lights_misc_status_label", subsys_grid.lights_misc_status_label);
                // builder->get_widget("navig_misc_status_label", ptz_misc_status_label);
            
            
            // builder->get_widget("image_draw_area", image_draw_area);
            //     image_draw_area->signal_draw().connect(sigc::mem_fun(*this, &ArmHMINode::handleVideoFrameDraw));
            // // changeCard("full_control_card");

            builder->get_widget("home_all_button", home_all_button);
                home_all_button->signal_clicked().connect(sigc::mem_fun(*this, &ArmHMINode::handleHomeAllButtonClick));
            builder->get_widget("pos_feed_on_button", pos_feed_on_button);
                pos_feed_on_button->signal_clicked().connect(sigc::mem_fun(*this, &ArmHMINode::handlePosFeedOnButtonClick));
            builder->get_widget("pos_feed_off_button", pos_feed_off_button);
                pos_feed_off_button->signal_clicked().connect(sigc::mem_fun(*this, &ArmHMINode::handlePosFeedOffButtonClick));
            builder->get_widget("test_limits_button", test_limits_button);
                test_limits_button->signal_clicked().connect(sigc::mem_fun(*this, &ArmHMINode::handleTestLimitsButtonClick));

            builder->get_widget("arm_abort_button", arm_abort_button);
                arm_abort_button->signal_clicked().connect(sigc::mem_fun(*this, &ArmHMINode::handleArmAbortButtonClick));      
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
                inc_axis_button[i]->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &ArmHMINode::handleIncAxisButtonClick), i));
                inc_axis_button[i]->signal_released().connect(sigc::mem_fun(*this, &ArmHMINode::handleAxisButtonRelease));
                dec_axis_button[i]->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &ArmHMINode::handleDecAxisButtonClick), i));
                dec_axis_button[i]->signal_released().connect(sigc::mem_fun(*this, &ArmHMINode::handleAxisButtonRelease));

            }

            builder->get_widget("axis_1_speed_spinbutton", axis_speed_spinbutton[0]);
            builder->get_widget("axis_2_speed_spinbutton", axis_speed_spinbutton[1]);
            builder->get_widget("axis_3_speed_spinbutton", axis_speed_spinbutton[2]);
            builder->get_widget("axis_4_speed_spinbutton", axis_speed_spinbutton[3]);
            builder->get_widget("axis_5_speed_spinbutton", axis_speed_spinbutton[4]);
            builder->get_widget("axis_6_speed_spinbutton", axis_speed_spinbutton[5]);
            for(int i = 0; i < 6; i++){
                axis_speed_spinbutton[i]->set_range(0.0, 90.0);
                axis_speed_spinbutton[i]->set_value(axis_hmi_speed[i]);
                axis_speed_spinbutton[i]->signal_value_changed().connect(sigc::bind(sigc::mem_fun(*this, &ArmHMINode::handleAxisSpeedUpdate) , i));
            }




            RCLCPP_INFO(this->get_logger(), "Meowing complete");

        }


    void run(){
            RCLCPP_INFO(this->get_logger(), "Meowing start");

        app->run(*arm_hmi_window);
            RCLCPP_INFO(this->get_logger(), "Meowing run");

    }


    Glib::RefPtr<Gtk::Application> app;
    // void load_css(const Glib::RefPtr<Gtk::CssProvider>& css_provider);
    //*Draw functions, can redraw widgets based on this node's data
    bool handleSubsystemStatusGridDraw(const Cairo::RefPtr<Cairo::Context>& context);
    std::string floatToStringTruncate(float value, int decimals);

    // bool handleVideoFrameDraw(const Cairo::RefPtr<Cairo::Context>& cr);
    // void image_feed_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    private:
    std::string arm_css_file_path;
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
    void handleAxisSpeedUpdate(int i);

    void handleIKTestButtonClick();
    
    // void armFeebackCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg);

    Gtk::Window* middle_window;
    Gtk::Window* arm_hmi_window;
    Gtk::Stack* middle_stack; //Like a deck of cards, each card is a different screen we can view

    Gtk::Widget* image_draw_area;

    Gtk::Label* axis_pos_label[NUM_JOINTS];

    Gtk::Button* home_all_button;
    Gtk::Button* pos_feed_on_button;
    Gtk::Button* pos_feed_off_button;
    Gtk::Button* test_limits_button;

    Gtk::SpinButton* axis_speed_spinbutton[NUM_JOINTS];


    Gtk::Button* arm_abort_button; //! Arm Abort Button

    Gtk::Button* dec_axis_button[NUM_JOINTS];
    Gtk::Button* inc_axis_button[NUM_JOINTS];
  
    float axis_hmi_speed[NUM_JOINTS] = {15.0, 12.0, 15.0, 30.0, 15.0, 80.0}; //Set axis default speeds here. In degrees per second


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

    // SubsystemGrid subsys_grid;


    









    std::string package_share_dir;
    
    rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr arm_status_sub;
    rclcpp::Publisher<rover_msgs::msg::ArmCommand>::SharedPtr arm_cmd_pub;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_feed_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_monitor_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr six_motor_monitor_sub;
    
};
