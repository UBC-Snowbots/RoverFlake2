#pragma once
#include <HMICommon.h>
#include <rover_msgs/msg/arm_command.hpp>
#include <rover_msgs/msg/science_module.hpp>
//#include <rover_msgs/msg/camera_video.hpp>

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
    //camera_video_pub = this->create_publisher<rover_msgs::msg::CameraVideo>("/science/camera_video", qos);
    

    // valve_feedback_sub = this->create_subscription<rover_msgs::msg::ScienceCommand>(
    //     "/science/feedback", qos, std::bind(&ScienceHMINode::valveFeedbackCallback, this, std::placeholders::_1));

    //* Grab the window (so GTK knows what to show)
    builder->get_widget("science_window", science_window);
    
    
    // **Text Input for Integer 0-15**
    // builder->get_widget("indexnumberentry", indexnumberentry);
    // indexnumberentry->signal_activate().connect(sigc::mem_fun(*this, &ScienceHMINode::handleTextboxInput));

    // **Button Handling**
    builder->get_widget("rinsebutton", rinsebutton);
    builder->get_widget("agitatorbutton", agitatorbutton);
    builder->get_widget("processbutton", processbutton);
    builder->get_widget("purgebutton", purgebutton);

    builder->get_widget("sv1button", sv1button);
    builder->get_widget("sv2button", sv2button);
    builder->get_widget("sv3button", sv3button);
    builder->get_widget("sv4button", sv4button);


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
    

    //builder->get_widget("p1button", p1button);
    builder->get_widget("p1reverse", p1reverse);
    builder->get_widget("p1stop", p1stop);
    builder->get_widget("p1forward", p1forward);
    builder->get_widget("pumpstatuslabel", pumpstatuslabel);


    builder->get_widget("osf1button", osf1button);
    builder->get_widget("osf2button", osf2button);
    builder->get_widget("statuslabel", statuslabel);
    builder->get_widget("ofsblockedlabel", ofsblockedlabel);

    builder->get_widget("prevIndexbutton", prevIndexbutton);
    builder->get_widget("nextidxbutton", nextidxbutton);
    builder->get_widget("resetbutton", resetbutton);
    builder->get_widget("label_carousel", label_carousel);
    builder->get_widget("smallbutton", smallbutton);
    builder->get_widget("largebutton", largebutton);

    builder->get_widget("spectrobutton", spectrobutton);
    builder->get_widget("light1button", light1button);
    builder->get_widget("light2button", light2button);
    builder->get_widget("agitatorpowerbutton", agitatorpowerbutton);


    builder->get_widget("camera1button", camera1button);
    builder->get_widget("cameraInput", cameraInput);

    builder->get_widget("estopbutton", estopbutton);



    //Constructor for sequence_buttons
    sequence_buttons = { rinsebutton, agitatorbutton, processbutton, purgebutton };

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
    
    pump_buttons = {p1reverse, p1stop, p1forward};


    p1reverse->signal_clicked().connect(sigc::bind(sigc::mem_fun(*this, &ScienceHMINode::setPump), true, static_cast<int>(PumpStatus::Reverse)));
    p1stop->signal_clicked().connect(sigc::bind(sigc::mem_fun(*this, &ScienceHMINode::setPump), true, static_cast<int>(PumpStatus::Stop)));
    p1forward->signal_clicked().connect(sigc::bind(sigc::mem_fun(*this, &ScienceHMINode::setPump), true, static_cast<int>(PumpStatus::Forward)));


    //p1button->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::P1clicked));
    osf1button->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::OSF1clicked));
    osf2button->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::OSF2clicked));

    prevIndexbutton->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::prevClicked));
    nextidxbutton->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::nextClicked));
    resetbutton->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::resetClicked));

    smallbutton->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::smallClicked));
    largebutton->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::largeClicked));

    spectrobutton->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::spectroClicked));
    agitatorpowerbutton->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::agPowerClicked));
    light1button->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::light1Clicked));
    light2button->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::ligth2Clicked));

    //camera1button->signal_clicked().connect(sigc::bind(sigc::mem_fun(*this, &ScienceHMINode::cameraFeedChosen), true, 1));
    //camera2button->signal_clicked().connect(sigc::bind(sigc::mem_fun(*this, &ScienceHMINode::cameraFeedChosen), true, 2));


    estopbutton->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::stopClicked));



    // camera1button->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::camera1clicked));
    // camera2button->signal_clicked().connect(sigc::mem_fun(*this, &ScienceHMINode::camera2clicked));





    //* css files
    // main_css_file_path = this->package_share_dir + "/css_files/science_style.css";
    // RCLCPP_INFO(this->get_logger(), main_css_file_path.c_str());
    // auto css_provider = Gtk::CssProvider::create();
    // load_css(css_provider, main_css_file_path);
    RCLCPP_INFO(this->get_logger(), "Meowing css");

    auto screen = Gdk::Screen::get_default();
    //auto style_context = get_style_context();
    //style_context->add_provider_for_screen(screen, css_provider, GTK_STYLE_PROVIDER_PRIORITY_USER);

    RCLCPP_INFO(this->get_logger(), "Meowing builder");

    // //* Setup GTK widgets
    // builder->get_widget("middle_window", middle_window);
    // builder->get_widget("middle_stack", middle_stack);
    // RCLCPP_INFO(this->get_logger(), "Meowing builder");

  }

  void run()
{
    RCLCPP_INFO(this->get_logger(), "Start");

    // Load and apply CSS before the UI starts
    auto cssProvider = Gtk::CssProvider::create();

    try {
        cssProvider->load_from_path("/home/ubcrover/RoverFlake2/src/rover_hmi/css_files/science_style.css");  // Make sure this path is correct
    } catch (const Glib::Error& ex) {
        std::cerr << "Failed to load CSS: " << ex.what() << std::endl;
    }

    auto screen = Gdk::Screen::get_default();
    Gtk::StyleContext::add_provider_for_screen(
      Gdk::Screen::get_default(),
      cssProvider,
      GTK_STYLE_PROVIDER_PRIORITY_APPLICATION  // 🔥 max override
  );
  
    // Now launch the GTK application
    app->run(*science_window);

    RCLCPP_INFO(this->get_logger(), "App Run Success");
}


  Glib::RefPtr<Gtk::Application> app;

private:
  std::string main_css_file_path;


  // in ScienceHMINode.h
  rover_msgs::msg::ScienceModule home_msg;

  int rinse_step;
  sigc::connection rinse_timer;

  int ag_step;
  sigc::connection ag_timer;


  int process_step;
  sigc::connection process_timer;



  std::string package_share_dir;


  // Carousel index input (0-15)
  int carousel_index_ = 0;
  Gtk::Window* science_window;

  Gtk::Button* sv1button;
  Gtk::Button* sv2button;
  Gtk::Button* sv3button;
  Gtk::Button* sv4button;

  Gtk::Button* p1reverse;
  Gtk::Button* p1forward;
  Gtk::Button* p1stop;
  Gtk::Label* pumpstatuslabel;
 
  Gtk::Button* osf1button;
  Gtk::Button* osf2button;
  Gtk::Label* statuslabel;
  Gtk::Label* ofsblockedlabel;



  Gtk::Button* rinsebutton;
  Gtk::Button* agitatorbutton;
  Gtk::Button* processbutton;
  Gtk::Button* purgebutton;

  Gtk::Label* label_carousel;
  Gtk::Button* prevIndexbutton;
  Gtk::Button* nextidxbutton;
  Gtk::Button* resetbutton;
  Gtk::Button* smallbutton;
  Gtk::Button* largebutton;

  Gtk::Button* agitatorpowerbutton;
  Gtk::Button* spectrobutton;
  Gtk::Button* light1button;
  Gtk::Button* light2button;

  Gtk::Button* camera1button;

  Gtk::Label* cameraInput;

  Gtk::Button* estopbutton;





    // // Store all valve buttons in a vector
    // std::vector<Gtk::Button*> valve_buttons;
    
    // // Generic button reference for easier styling
    // Gtk::Button* valve_button;

  void SV1clicked();
  void SV2clicked();
  void SV3clicked();
  void SV4clicked();


  int toggleButtonStyle(Gtk::Button* btn, const std::string& active_class, const std::string& inactive_class);


  void setSequence(bool pressed, int button); //RECHECK
  void handleTextboxInput();

  //void setCarouselIndex(int index);

  void setPump(bool pressed, int button);
  
  //void P1clicked();
  void OSF1clicked();
  void OSF2clicked();
  void updateStatusLabel();
  void updateOFSBlockedLable();

  void prevClicked();
  void nextClicked();
  void updateCarouselIndexLabel();
  void resetClicked();
  void smallClicked();
  void largeClicked();

  void spectroClicked();
  void agPowerClicked();
  void light1Clicked();
  void ligth2Clicked();


  //void cameraFeedChosen(bool clicked, int id);
  void cameraInputLabel();

  void stopClicked();


  void resetSystem();
  void rinseSequence();
  void agitatorSequence();
  void processSequence();
  void purgeSequence();
  void updateValveButton(Gtk::Button* button, bool energized);
   void updateAGButton(Gtk::Button* button, bool energized);

  void updatePumpUI(Gtk::Label* label, Gtk::Button* forwardButton, int pumpStatus);




  // Store sequence buttons for resetting styles
  std::vector<Gtk::Button*> sequence_buttons;

  enum class sequence_status
  {
    rinse,
    agitator,
    process,
    purge
  };

  // Store sequence buttons for resetting styles
  std::vector<Gtk::Button*> pump_buttons;

  enum class PumpStatus {
    Reverse,
    Stop, 
    Forward
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

  //rclcpp::Publisher<rover_msgs::msg::CameraVideo>::SharedPtr camera_video_pub;


  //rclcpp::Publisher<rover_msgs::msg::CameraCommand>::SharedPtr camera_pub;
 // rclcpp::Publisher<std_msgs::msg::int16>::SharedPtr camera_fd;


 
};
