#pragma once
#include <HMICommon.h>
#include <rover_msgs/msg/arm_command.hpp>
#include <rover_msgs/msg/science_module.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <arm_hardware_interface/ArmSerialProtocol.h>  //? Shouldn't be included here, but leave it for now. armControlParams is meant to be the common header for all arm parameters.
#include <arm_control/include/armControlParams.h>

// opencv and image processing
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>


// standard c++ stuff, some may alreaddy be included in rclcpp
#include <chrono>
#include <ctime>
#include <memory>
#include <thread>
// #include <signal.h>

#define QUEUE_SIZE 20
#define RCL_SPIN_RATE 30

class ScienceHMINode : public rclcpp::Node, public Gtk::Window
{
public:

  ScienceHMINode() : Node("science_hmi_node")
  {
    set_title("Science Module HMI");

    this->package_share_dir = ament_index_cpp::get_package_share_directory("rover_hmi");

    std::string glade_file_path = this->package_share_dir + "/glade_files/science_module_hmi.glade";
    RCLCPP_INFO(this->get_logger(), glade_file_path.c_str());
    auto builder = Gtk::Builder::create_from_file(glade_file_path.c_str());
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    
    // **ROS2 Communication**
    valve_pub = this->create_publisher<rover_msgs::msg::ScienceModule>("/science/command", qos);
    sequence_pub = this->create_publisher<rover_msgs::msg::ScienceModule>("/science/command", qos);
    carousel_pub = this->create_publisher<rover_msgs::msg::ScienceModule>("/science/command", qos);
    pump_pub = this->create_publisher<rover_msgs::msg::ScienceModule>("/science/command", qos);
    osf_pub = this->create_publisher<rover_msgs::msg::ScienceModule>("/science/command", qos);
    agitator_pub = this->create_publisher<rover_msgs::msg::ScienceModule>("/science/command", qos);
    spectro_pub = this->create_publisher<rover_msgs::msg::ScienceModule>("/science/command", qos);
    light_pub = this->create_publisher<rover_msgs::msg::ScienceModule>("/science/command", qos);

    // valve_feedback_sub = this->create_subscription<rover_msgs::msg::ScienceCommand>(
    //     "/science/feedback", qos, std::bind(&ScienceHMINode::valveFeedbackCallback, this, std::placeholders::_1));

    
    
    // **Text Input for Integer 0-15**
    builder->get_widget("indexnumberentry", indexnumberentry);
    indexnumberentry->signal_changed().connect(sigc::mem_fun(*this, &ScienceHMINode::handleTextboxInput));

    // **Button Handling**
    builder->get_widget("rinsebutton", rinsebutton);
    builder->get_widget("agitatorbutton", agitatorbutton);
    builder->get_widget("processbutton", processbutton);
    builder->get_widget("purgebutton", purgebutton);

    builder->get_widget("sv1button", sv1button);
    builder->get_widget("sv2button", sv2button);
    builder->get_widget("sv3button", sv3button);
    builder->get_widget("svf1button", svf1button);
    builder->get_widget("svf2button", svf2button);


    // std::vector<std::string> valve_ids = {
    //   "sv1button", "sv2button", "sv3button", "sv4button", "svf1button", "svf2button"
    // };
  
    // for (const auto& id : valve_ids) {
    //     Gtk::Button* button = nullptr;
    //     builder->get_widget(id, button);
    //     if (button) {
    //         valve_buttons.push_back(button);
    //         button->signal_clicked().connect(sigc::bind(sigc::mem_fun(*this, &ScienceHMINode::toggleValve), button));
    //     }
    // }
    

    builder->get_widget("p1button", p1button);
    builder->get_widget("osf1button", osf1button);
    builder->get_widget("osf2button", osf2button);

    builder->get_widget("prevIndexbutton", prevIndexbutton);
    builder->get_widget("nextidxbutton", nextidxbutton);
    builder->get_widget("smallbutton", smallbutton);
    builder->get_widget("largebutton", largebutton);

    builder->get_widget("spectrobutton", spectrobutton);
    builder->get_widget("light1button", light1button);
    builder->get_widget("light2button", light2button);
    builder->get_widget("agitatorpowerbutton", agitatorpowerbutton);


    //Constructor for sequence_buttons
    sequence_buttons = { rinsebutton };

    rinsebutton->signal_clicked().connect(sigc::bind(sigc::mem_fun(*this, &ScienceHMINode::setSequence), 
                                                        true, static_cast<int>(sequence_status::rinse)));
    agitatorbutton->signal_clicked().connect(sigc::bind(sigc::mem_fun(*this, &ScienceHMINode::setSequence), 
                                                        true, static_cast<int>(sequence_status::agitator)));
    processbutton->signal_clicked().connect(sigc::bind(sigc::mem_fun(*this, &ScienceHMINode::setSequence), 
                                                        true, static_cast<int>(sequence_status::process)));
    purgebutton->signal_clicked().connect(sigc::bind(sigc::mem_fun(*this, &ScienceHMINode::setSequence), 
                                                        true, static_cast<int>(sequence_status::purge)));
    
    sv1button->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::SV1clicked));
    sv2button->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::SV2clicked));
    sv3button->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::SV3clicked));
    sv4button->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::SV4clicked));
    svf1button->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::SVF1clicked));
    svf2button->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::SVF2clicked));


    p1button->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::P1clicked));
    osf1button->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::OSF1clicked));
    osf2button->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::OSF2clicked));

    prevIndexbutton->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::prevClicked));
    nextidxbutton->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::nextClicked));
    smallbutton->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::smallClicked));
    largebutton->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::largeClicked));

    spectrobutton->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::spectroClicked));
    agitatorpowerbutton->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::agPowerClicked));
    light1button->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::light1Clicked));
    light2button->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::ligth2Clicked));


    //* css files
    main_css_file_path = this->package_share_dir + "/css_files/science_style.css";
    RCLCPP_INFO(this->get_logger(), main_css_file_path.c_str());
    auto css_provider = Gtk::CssProvider::create();
    load_css(css_provider, main_css_file_path);
    RCLCPP_INFO(this->get_logger(), "Meowing css");

    auto screen = Gdk::Screen::get_default();
    auto style_context = get_style_context();
    style_context->add_provider_for_screen(screen, css_provider, GTK_STYLE_PROVIDER_PRIORITY_USER);
    RCLCPP_INFO(this->get_logger(), "Meowing builder");

    // //* Setup GTK widgets
    // builder->get_widget("middle_window", middle_window);
    // builder->get_widget("middle_stack", middle_stack);
    // RCLCPP_INFO(this->get_logger(), "Meowing builder");

  }

  void run()
  {
    RCLCPP_INFO(this->get_logger(), "Start");

    app->run(*this);
    RCLCPP_INFO(this->get_logger(), "App Run Success");
  }

  Glib::RefPtr<Gtk::Application> app;

private:
  std::string main_css_file_path;


  std::string package_share_dir;


  // Carousel index input (0-15)
  int carousel_index_;

  Gtk::Button* sv1button;
  Gtk::Button* sv2button;
  Gtk::Button* sv3button;
  Gtk::Button* sv4button;
  Gtk::Button* svf1button;
  Gtk::Button* svf2button;

  Gtk::Button* p1button;
  Gtk::Button* osf1button;
  Gtk::Button* osf2button;


  Gtk::Button* rinsebutton;
  Gtk::Button* agitatorbutton;
  Gtk::Button* processbutton;
  Gtk::Button* purgebutton;


  Gtk::Entry* indexnumberentry;
  Gtk::Button* prevIndexbutton;
  Gtk::Button* nextidxbutton;
  Gtk::Button* smallbutton;
  Gtk::Button* largebutton;

  Gtk::Button* agitatorpowerbutton;
  Gtk::Button* spectrobutton;
  Gtk::Button* light1button;
  Gtk::Button* light2button;



    // // Store all valve buttons in a vector
    // std::vector<Gtk::Button*> valve_buttons;
    
    // // Generic button reference for easier styling
    // Gtk::Button* valve_button;

  void SV1clicked();
  void SV2clicked();
  void SV3clicked();
  void SV4clicked();
  void SVF1clicked();
  void SVF2clicked();

  void setSequence(bool pressed, int button); //RECHECK
  void handleTextboxInput();

  void setCarouselIndex(int index);

  void P1clicked();
  void OSF1clicked();
  void OSF2clicked();

  void prevClicked();
  void nextClicked();
  void smallClicked();
  void largeClicked();

  void spectroClicked();
  void agPowerClicked();
  void light1Clicked();
  void ligth2Clicked();



  // Store sequence buttons for resetting styles
  std::vector<Gtk::Button*> sequence_buttons;

  enum class sequence_status
  {
    rinse,
    agitator,
    process,
    purge
  };


  // Publishers **NEED HELP HERE
  rclcpp::Publisher<rover_msgs::msg::ScienceModule>::SharedPtr sequence_pub;
  rclcpp::Publisher<rover_msgs::msg::ScienceModule>::SharedPtr valve_pub;
  rclcpp::Publisher<rover_msgs::msg::ScienceModule>::SharedPtr carousel_pub;
  rclcpp::Publisher<rover_msgs::msg::ScienceModule>::SharedPtr pump_pub;
  rclcpp::Publisher<rover_msgs::msg::ScienceModule>::SharedPtr osf_pub;
  rclcpp::Publisher<rover_msgs::msg::ScienceModule>::SharedPtr agitator_pub;
  rclcpp::Publisher<rover_msgs::msg::ScienceModule>::SharedPtr spectro_pub;
  rclcpp::Publisher<rover_msgs::msg::ScienceModule>::SharedPtr light_pub;





 
};
