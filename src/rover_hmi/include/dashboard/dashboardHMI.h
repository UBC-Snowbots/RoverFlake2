#include <HMICommon.h>
#include <rover_msgs/msg/sub_system_health.hpp>
#define NUM_MONITORED_SYSTEMS 6
#define RUN 0xA
#define KILL 0xB
// #include <helper_functions.h>

//? Watchdog stuffs..?
#include <unistd.h>     // For fork, execlp
#include <sys/types.h>  // For pid_t
#include <stdlib.h>     // For exit


class DashboardHMINode : public rclcpp::Node, public Gtk::Window
{
public:
    DashboardHMINode() : Node("dashboard_hmi_node")
    {
        set_title("Rover Dashboard"); //set the app/window title
        //* Set up pubs n subs
        heartbeat_monitor_sub = this->create_subscription<rover_msgs::msg::SubSystemHealth>(
          "/system/heartbeats", 10, std::bind(&DashboardHMINode::heartbeatCallback, this, std::placeholders::_1));
        monitored_systems["control_base"];

        this->package_share_dir = ament_index_cpp::get_package_share_directory("rover_hmi");
        std::string glade_file_path = this->package_share_dir + "/glade_files/dashboard.glade";
        auto builder = Gtk::Builder::create_from_file(glade_file_path.c_str());
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

        auto screen = Gdk::Screen::get_default();
        main_css_file_path = this->package_share_dir + "/css_files/main_style.css";
        auto css_provider = Gtk::CssProvider::create();
        load_css(css_provider, main_css_file_path);
        auto style_context = get_style_context();
        style_context->add_provider_for_screen(screen, css_provider, GTK_STYLE_PROVIDER_PRIORITY_USER);

          #include "gtksetup.h"
        
      builder->get_widget("global_msg_box", global_msg_label);
                global_msg_label->set_label("DASHBOARD STARTING...");

        RCLCPP_INFO(this->get_logger(), "BUILDER SUCCESS");
        global_msg_label->set_label("DASHBOARD STARTED");


    }

    ~DashboardHMINode(){
        std::system("notify-send DASHBOARD_OFFLINE going down with regular cleanup!");
        //! need to add orphan cleanup here
        // for(const auto &pair : running_child_pids){
        //   RCLCPP_INFO(this->get_logger(), "Performing standard execution of child: %s, with pid %d", pair.first, pair.second);
        //   // killChildPID(pair.second);
        //   // std::vector<pid_t> pids = getPidsByName(pair.first);
        //   // for(const pid_t pid : pids){
        //     // kill(pid, SIGINT); 
        //     killProcessGroup(pair.second);
        //   // }
        // }
        // RCLCPP_INFO(this->get_logger(), "Waiting for children to say their last words...");
        // sleep(5);
        // RCLCPP_INFO(this->get_logger(), "Times up, forcefully killing any slow orphans");
        // for(const auto &pair : running_child_pids){
        //   RCLCPP_INFO(this->get_logger(), "Performing standard execution of child: %s, with pid %d", pair.first, pair.second);
        //   // killChildPID(pair.second);
        //   // kill(pair.second, SIGKILL);
        //   killProcessGroup(pair.second);
        // }
      for(const auto &pair : monitored_systems){
        RCLCPP_INFO(this->get_logger(), "Performing standard execution of child: %s, with gpid %d", pair.first, pair.second.gpid);
        killSubSystem(pair.first);
      }

    }
    void run()
    {
      RCLCPP_INFO(this->get_logger(), "Start");
  
      app->run(*dash_window);
      RCLCPP_INFO(this->get_logger(), "App Run Success");
    }
    Glib::RefPtr<Gtk::Application> app;

private:
    std::string main_css_file_path;
    std::string package_share_dir;
    std::vector<std::string> monitored_systems_names = {"control_base", "drive_control", "arm_hardware", "arm_control", "science", "perceptions"};
    struct MonitoredSystem{
      std::string name;
      pid_t pid;
      pid_t gpid;
      pid_t sid;
      bool online = false;
    };
    // MonitoredSystem control_base_sys;
    std::unordered_map<std::string, MonitoredSystem> monitored_systems;
 
    
    // std::map

    //? Gtk Stuffs
    #include "gtkwidgets.h"
      SubSysStatusGrid system_health;


  //* Button callbacks, either to be triggered by a button in the HMI or a control base panel callback
  void subsystemRequest(int system_index, int request);


  //? Ros2 stuffs
 rclcpp::Subscription<rover_msgs::msg::SubSystemHealth>::SharedPtr heartbeat_monitor_sub;
  void heartbeatCallback(const rover_msgs::msg::SubSystemHealth::SharedPtr msg);
//  rclcpp::Publishe?



  //? watchdog stuffs
  std::unordered_map<std::string, pid_t> running_child_pids;
    //* CHILD PROCESSES
    void killSubSystem(std::string subsystem_name);
    void killProcessGroup(pid_t pgid);
    void runChildNode(std::string pkg, std::string node_or_launch_file, std::string subsytem_name, bool launch = false, bool kill_orphan = true);
    void killChildPID(pid_t target_pid);
    std::vector<pid_t> getPidsByName(const std::string &processName, bool verbose = false);

};