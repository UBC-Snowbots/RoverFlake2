#pragma once
#include <HMICommon.h>
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

    this->package_share_dir = ament_index_cpp::get_package_share_directory("science_module_hmi");

    std::string glade_file_path = this->package_share_dir + "/glade_files/science_module_hmis.glade";
    RCLCPP_INFO(this->get_logger(), glade_file_path.c_str());
    auto builder = Gtk::Builder::create_from_file(glade_file_path.c_str());
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    
    // **ROS2 Communication**
    valve_cmd_pub = this->create_publisher<rover_msgs::msg::ScienceCommand>("/science/command", qos);
    sequence_cmd_pub = this->create_publisher<rover_msgs::msg::ScienceCommand>("/science/command", qos);
    // valve_feedback_sub = this->create_subscription<rover_msgs::msg::ScienceCommand>(
    //     "/science/feedback", qos, std::bind(&ScienceHMINode::valveFeedbackCallback, this, std::placeholders::_1));

    
    
    // **Text Input for Integer 0-15**
    builder->get_widget("indexnum_textbox", valve_input_textbox);
    valve_input_textbox->signal_changed().connect(sigc::mem_fun(*this, &ScienceHMINode::setCarouselIndex));

    // **Button Handling**
    builder->get_widget("rinsesequence_button", rinsesequence_button);
    builder->get_widget("sv1_button", sv1_button);

    //Constructor for sequence_buttons
    sequence_buttons = { rinsesequence_button };

    rinsesequence_button->signal_clicked().connect(sigc::bind(sigc::mem_fun(*this, &ScienceHMINode::setSequence), 
                                                        static_cast<int>(sequence_status::rinse)));
    
    sv1_button->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::SV1clicked));


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

private:
  std::string main_css_file_path;
  Glib::RefPtr<Gtk::Application> app;


  // Carousel index input (0-15)
  int carousel_index_;

  Gtk::Button* sv1_button;
  Gtk::Button* rinsesequence_button;
  Gtk::Entry* valve_input_textbox;


  void SV1clicked();
  void setSequence(SequenceType); //RECHECK
  void handleTextboxInput();

  void setCarouselIndex(int index);

  // Store sequence buttons for resetting styles
  std::vector<Gtk::Button*> sequence_buttons;

  enum class sequence_status
  {
    rinse,
    agitator,
    orocess,
    purge
  };

  // Publishers **NEED HELP HERE
  rclcpp::Publisher<rover_msgs::msg::ScienceCommand>::SharedPtr sequence_pub_;
  rclcpp::Publisher<rover_msgs::msg::ScienceCommand>::SharedPtr valve_pub_;
  rclcpp::Publisher<rover_msgs::msg::ScienceCommand>::SharedPtr carousel_pub_;
 
};
