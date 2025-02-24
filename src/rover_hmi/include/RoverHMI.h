#pragma once
#include <gtkmm.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rover_msgs/msg/arm_command.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <arm_hardware_interface/ArmSerialProtocol.h>  //? Shouldn't be included here, but leave it for now. armControlParams is meant to be the common header for all arm parameters.
#include <arm_control/include/armControlParams.h>

// opencv and image processing
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <sstream>
#include <iomanip>  // For std::setprecision

// Global functions
void load_css(const Glib::RefPtr<Gtk::CssProvider>& provider, std::string css_file_path);

// standard c++ stuff, some may alreaddy be included in rclcpp
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
  MainHMINode() : Node("main_hmi_node")
  {
    set_title("middle");

    this->package_share_dir = ament_index_cpp::get_package_share_directory("rover_hmi");

    std::string glade_file_path = this->package_share_dir + "/glade_files/middle_screen.glade";
    RCLCPP_INFO(this->get_logger(), glade_file_path.c_str());
    auto builder = Gtk::Builder::create_from_file(glade_file_path.c_str());
    // auto qos = rclcpp::QoS(rclcpp::KeepLast(QUEUE_SIZE)).transient_local();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    arm_status_sub = this->create_subscription<rover_msgs::msg::ArmCommand>(
        "/arm/feedback", qos, std::bind(&MainHMINode::armFeedbackCallback, this, std::placeholders::_1));
    arm_cmd_pub = this->create_publisher<rover_msgs::msg::ArmCommand>("/arm/command", qos);

    image_feed_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/image_raw", 10, std::bind(&MainHMINode::image_feed_callback, this, std::placeholders::_1));

    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", qos);    
        arm_ik_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/arm_moveit_control/delta_twist_cmds", qos);  
    cmd_vel_monitor_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&MainHMINode::cmdVelCallback, this, std::placeholders::_1));



    //* Timers
    ik_timer = this->create_wall_timer(
         std::chrono::milliseconds(34),  // Timer interval
                std::bind(&MainHMINode::ik_timer_callback, this) // Callback function
                );

    ik_timer->cancel(); //Stop the timer as it defaults to started

    //* css files
    main_css_file_path = this->package_share_dir + "/css_files/main_style.css";
    RCLCPP_INFO(this->get_logger(), main_css_file_path.c_str());
    auto css_provider = Gtk::CssProvider::create();
    load_css(css_provider, main_css_file_path);
    RCLCPP_INFO(this->get_logger(), "Meowing css");

    auto screen = Gdk::Screen::get_default();
    auto style_context = get_style_context();
    style_context->add_provider_for_screen(screen, css_provider, GTK_STYLE_PROVIDER_PRIORITY_USER);
    RCLCPP_INFO(this->get_logger(), "Meowing builder");

    //* Setup GTK widgets
    builder->get_widget("middle_window", middle_window);
    builder->get_widget("middle_stack", middle_stack);
    // RCLCPP_INFO(this->get_logger(), "Meowing builder");



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

    builder->get_widget("forward_button", forward_button);
    forward_button->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleCmdVelButton), true,
                                                        static_cast<int>(cmd_vel_buttons::forward)));
    forward_button->signal_released().connect(sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleCmdVelButton), false,
                                                         static_cast<int>(cmd_vel_buttons::forward)));
    builder->get_widget("left_forward_button", left_forward_button);
    left_forward_button->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleCmdVelButton), true,
                                                        static_cast<int>(cmd_vel_buttons::left_forward)));
    left_forward_button->signal_released().connect(sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleCmdVelButton), false,
                                                         static_cast<int>(cmd_vel_buttons::left_forward)));

    builder->get_widget("right_forward_button", right_forward_button);
    right_forward_button->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleCmdVelButton), true,
                                                        static_cast<int>(cmd_vel_buttons::right_forward)));
    right_forward_button->signal_released().connect(sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleCmdVelButton), false,
                                                         static_cast<int>(cmd_vel_buttons::right_forward)));

    builder->get_widget("rotate_ccw_button", rotate_ccw_button);
    rotate_ccw_button->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleCmdVelButton), true,
                                                        static_cast<int>(cmd_vel_buttons::rotate_ccw)));
    rotate_ccw_button->signal_released().connect(sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleCmdVelButton), false,
                                                         static_cast<int>(cmd_vel_buttons::rotate_ccw)));

    builder->get_widget("rotate_cw_button", rotate_cw_button);
    rotate_cw_button->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleCmdVelButton), true,
                                                        static_cast<int>(cmd_vel_buttons::rotate_cw)));
    rotate_cw_button->signal_released().connect(sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleCmdVelButton), false,
                                                         static_cast<int>(cmd_vel_buttons::rotate_cw)));

    builder->get_widget("reverse_button", reverse_button);
    reverse_button->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleCmdVelButton), true,
                                                        static_cast<int>(cmd_vel_buttons::reverse)));
    reverse_button->signal_released().connect(sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleCmdVelButton), false,
                                                         static_cast<int>(cmd_vel_buttons::reverse)));

    builder->get_widget("left_reverse_button", left_reverse_button);
    left_reverse_button->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleCmdVelButton), true,
                                                        static_cast<int>(cmd_vel_buttons::left_reverse)));
    left_reverse_button->signal_released().connect(sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleCmdVelButton), false,
                                                         static_cast<int>(cmd_vel_buttons::left_reverse)));

    builder->get_widget("right_reverse_button", right_reverse_button);
    right_reverse_button->signal_pressed().connect(sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleCmdVelButton), true,
                                                        static_cast<int>(cmd_vel_buttons::right_reverse)));
    right_reverse_button->signal_released().connect(sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleCmdVelButton), false,
                                                         static_cast<int>(cmd_vel_buttons::right_reverse)));

    //* Sliders for cmd vel tester
    builder->get_widget("linear_magslider", linear_magslider);
    builder->get_widget("angular_magslider", angular_magslider);

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

    builder->get_widget("inc_linx_button", inc_ik_button[0]);
    builder->get_widget("inc_liny_button", inc_ik_button[1]);
    builder->get_widget("inc_linz_button", inc_ik_button[2]);
    builder->get_widget("inc_angx_button", inc_ik_button[3]);
    builder->get_widget("inc_angy_button", inc_ik_button[4]);
    builder->get_widget("inc_angz_button", inc_ik_button[5]);

    builder->get_widget("dec_linx_button", dec_ik_button[0]);
    builder->get_widget("dec_liny_button", dec_ik_button[1]);
    builder->get_widget("dec_linz_button", dec_ik_button[2]);
    builder->get_widget("dec_angx_button", dec_ik_button[3]);
    builder->get_widget("dec_angy_button", dec_ik_button[4]);
    builder->get_widget("dec_angz_button", dec_ik_button[5]);

    builder->get_widget("inc_ee_button", inc_ee_button);
    builder->get_widget("dec_ee_button", dec_ee_button);
    inc_ee_button->signal_pressed().connect(
          sigc::mem_fun(*this, &MainHMINode::handleIncEEButtonClick));
    inc_ee_button->signal_released().connect(sigc::mem_fun(*this, &MainHMINode::handleAxisButtonRelease));
    dec_ee_button->signal_pressed().connect(
         sigc::mem_fun(*this, &MainHMINode::handleDecEEButtonClick));
    dec_ee_button->signal_released().connect(sigc::mem_fun(*this, &MainHMINode::handleAxisButtonRelease));
     

    for (int i = 0; i < 6; i++)
    {
      inc_axis_button[i]->signal_pressed().connect(
          sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleIncAxisButtonClick), i));
      inc_axis_button[i]->signal_released().connect(sigc::mem_fun(*this, &MainHMINode::handleAxisButtonRelease));
      dec_axis_button[i]->signal_pressed().connect(
          sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleDecAxisButtonClick), i));
      dec_axis_button[i]->signal_released().connect(sigc::mem_fun(*this, &MainHMINode::handleAxisButtonRelease));
      inc_ik_button[i]->signal_pressed().connect(
          sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleIncIKButtonClick), i));
      inc_ik_button[i]->signal_released().connect(sigc::mem_fun(*this, &MainHMINode::handleIKButtonRelease));
      dec_ik_button[i]->signal_pressed().connect(
          sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleDecIKButtonClick), i));
      dec_ik_button[i]->signal_released().connect(sigc::mem_fun(*this, &MainHMINode::handleIKButtonRelease));
    }

    builder->get_widget("axis_1_speed_spinbutton", axis_speed_spinbutton[0]);
    builder->get_widget("axis_2_speed_spinbutton", axis_speed_spinbutton[1]);
    builder->get_widget("axis_3_speed_spinbutton", axis_speed_spinbutton[2]);
    builder->get_widget("axis_4_speed_spinbutton", axis_speed_spinbutton[3]);
    builder->get_widget("axis_5_speed_spinbutton", axis_speed_spinbutton[4]);
    builder->get_widget("axis_6_speed_spinbutton", axis_speed_spinbutton[5]);

    builder->get_widget("linx_speed_spinbutton", ik_speed_spinbutton[0]);
    builder->get_widget("liny_speed_spinbutton", ik_speed_spinbutton[1]);
    builder->get_widget("linz_speed_spinbutton", ik_speed_spinbutton[2]);
    builder->get_widget("angx_speed_spinbutton", ik_speed_spinbutton[3]);
    builder->get_widget("angy_speed_spinbutton", ik_speed_spinbutton[4]);
    builder->get_widget("angz_speed_spinbutton", ik_speed_spinbutton[5]);
    for (int i = 0; i < 6; i++)
    {
      axis_speed_spinbutton[i]->set_range(0.0, MainHMINode::max_speeds[i]);
      axis_speed_spinbutton[i]->set_value(5.0);
      axis_speed_spinbutton[i]->signal_value_changed().connect(
          sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleAxisSpeedUpdate), i));
      ik_speed_spinbutton[i]->set_range(0.0, MainHMINode::max_ik_speed);
      ik_speed_spinbutton[i]->set_value(ik_hmi_speed[i]);
      ik_speed_spinbutton[i]->signal_value_changed().connect(
          sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleIKSpeedUpdate), i));
    }

    builder->get_widget("next_panel_button", next_panel_button);
    next_panel_button->signal_clicked().connect(
        sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleCardButtonSwitch), true));
    builder->get_widget("prev_panel_button", prev_panel_button);
    prev_panel_button->signal_clicked().connect(
        sigc::bind(sigc::mem_fun(*this, &MainHMINode::handleCardButtonSwitch), false));

    RCLCPP_INFO(this->get_logger(), "Builder complete");
  }

  void run()
  {
    RCLCPP_INFO(this->get_logger(), "Start");

    app->run(*middle_window);
    RCLCPP_INFO(this->get_logger(), "App Run Success");
  }
  std::string current_middle_card = "system_overview_card";
  // void changeCard(cards target_card);
  void changeCard(std::string target_card);

  Glib::RefPtr<Gtk::Application> app;
  // void load_css(const Glib::RefPtr<Gtk::CssProvider>& css_provider);
  //*Draw functions, can redraw widgets based on this node's data
  bool handleSubsystemStatusGridDraw(const Cairo::RefPtr<Cairo::Context>& context);
  std::string floatToStringTruncate(float value, int decimals);

  bool handleVideoFrameDraw(const Cairo::RefPtr<Cairo::Context>& cr);
  void image_feed_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  enum class cards
  {  //? HMI functions like a stack of cards, every card is a different panel we can view. HMI can only show one card at
     //a time.
    system_overview,
    full_control,
    arm_testing,                //? Panel used for running the arm
    control_base_testing_card,  //? Panel used for testing control base buttons. Akin to like a keyboard macro tester
    cmd_vel_testing_card,
    //? add a new card here if needed

    num_cards  //! Always keep at the end of this enumerator.
  };
  const std::string available_cards[static_cast<int>(cards::num_cards)] = {
    "system_overview_card", "full_control_card", "arm_testing_card", "control_base_testing_card",
    "cmd_vel_testing_card"
  };  //? Manually set, refer to glade/gtk widget ID

private:
  std::string main_css_file_path;

  int current_card_i = 0;
  std::string current_card = available_cards[0];
  void armFeedbackCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg);
  // rclcpp::TimerBase
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void handleHomeAllButtonClick();
  void handlePosFeedOnButtonClick();
  void handlePosFeedOffButtonClick();
  void handleIncAxisButtonClick(int index);  // RELATIVE VELOCITIERSs
  void handleDecAxisButtonClick(int index);
  void handleAxisButtonRelease();

  void handleDecEEButtonClick();
  void handleIncEEButtonClick();

  void handleIncIKButtonClick(int index);  // RELATIVE VELOCITIERSs
  void handleDecIKButtonClick(int index);
  void handleIKButtonRelease();

 
  void handleArmAbortButtonClick();
  void handleTestLimitsButtonClick();
  void handleAxisSpeedUpdate(int i);
  void handleIKSpeedUpdate(int i);
  void handleCardButtonSwitch(bool next);

  void handleIKTestButtonClick();
  // void armFeebackCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg);

  Gtk::Window* middle_window;
  Gtk::Stack* middle_stack;  // Like a deck of cards, each card is a different screen we can view

  Gtk::Widget* image_draw_area;

  Gtk::Label* axis_pos_label[6];

  Gtk::Button* home_all_button;
  Gtk::Button* next_panel_button;
  Gtk::Button* prev_panel_button;
  Gtk::Button* pos_feed_on_button;
  Gtk::Button* pos_feed_off_button;
  Gtk::Button* test_limits_button;

  //* CMD VEL TESTING PANEL
  void handleCmdVelButton(bool pressed, int button);
  Gtk::Button* forward_button;
  Gtk::Button* reverse_button;
  Gtk::Button* rotate_ccw_button;
  Gtk::Button* rotate_cw_button;
  Gtk::Button* right_forward_button;
  Gtk::Button* left_forward_button;
  Gtk::Button* left_reverse_button;
  Gtk::Button* right_reverse_button;

  Gtk::Scale* linear_magslider;
  Gtk::Scale* angular_magslider;

  enum class cmd_vel_buttons
  {
    forward,
    reverse,
    rotate_ccw,
    rotate_cw,
    right_forward,
    left_forward,
    left_reverse,
    right_reverse
  };

  //* Arm Control Panel
  Gtk::SpinButton* axis_speed_spinbutton[6];
  Gtk::SpinButton* ik_speed_spinbutton[6];

  Gtk::Button* arm_abort_button;  //! Arm Abort Button

  Gtk::Button* dec_axis_button[6];
  Gtk::Button* inc_axis_button[6];

  Gtk::Button* dec_ik_button[6];
  Gtk::Button* inc_ik_button[6];

  Gtk::Button* dec_ee_button;
  Gtk::Button* inc_ee_button;

  float ee_speed = 50.0;

  float axis_hmi_speed[6] = { 5.0, 5.0, 5.0, 5.0, 5.0, 5.0 };
  float ik_hmi_speed[6] = { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 };
  float max_speeds[NUM_JOINTS] = { 90.0, 60.0, 90.0, 100.0, 100.0, 180.0 };
  float max_ik_speed = 1;

  //* System Overview
  struct SubsystemGrid
  {
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
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr arm_ik_pub;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_feed_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_monitor_sub;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr six_motor_monitor_sub;

  cv::Mat image_;
  Glib::RefPtr<Gdk::Pixbuf> pixbuf_;
  std::mutex image_mutex_;

  void ik_timer_callback();
  //in private:
  rclcpp::TimerBase::SharedPtr ik_timer; // Timer handle
  int current_ik_index = -1;
  float current_ik_value = 0.2;
 
};

/* Topics to sub to

    - Joint states. if we want arm stuff, or pure rviz?
    - Enviroment data
    - BMS data







 */