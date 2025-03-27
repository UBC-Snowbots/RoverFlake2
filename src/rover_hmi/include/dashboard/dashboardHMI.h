#include <HMICommon.h>
#include <rover_msgs/msg/sub_system_health.hpp>
#include <rover_msgs/msg/heart_request.hpp>
#include "dashboardDefinitions.h"

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
        // onboard_heart_request_pub = this->create_publisher<
        //TODO get params in from heart yamls, and ready the publishers on the right topics
        //TODO then connect buttons to publish
        //TODO then create a proper feedback using ros2 node or heartbeats?
        heartbeat_monitor_sub = this->create_subscription<rover_msgs::msg::SubSystemHealth>(
          "/system/heartbeats", 10, std::bind(&DashboardHMINode::heartbeatCallback, this, std::placeholders::_1));
        
          //* Control Base SubSystem
            SubSystemProcess cbs_background;
              cbs_background.type = LAUNCHFILE;
              cbs_background.pkg = "rover_launchers";
              cbs_background.exec = "cbs_bringup.launch.py";
              monitored_systems[monitored_system_names[0]].process = cbs_background;
            SubSystemProcess drive_control;
              drive_control.type = LAUNCHFILE;
              drive_control.pkg = "drive_control";
              drive_control.exec = "drive_control_on_board.launch.py";
              monitored_systems[monitored_system_names[1]].process = drive_control;
            SubSystemProcess xx;
              xx.type = LAUNCHFILE;
              xx.pkg = "xx";
              xx.exec = "xx";
              monitored_systems[monitored_system_names[2]].process = xx;
            // monitored_systems["control_base"].processes.push_back(cbs_background); //! only one process..
          






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
        killSubSystem(pair.first); //will send SIGTERM
      }
      //! may need to add SIGKILL signal

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
    
    struct SubSystemProcess{
      int type = LAUNCHFILE;
      std::string pkg;
      std::string exec;
    };
    
    
    struct MonitoredSystem{ //* might want to make a class in the future.. assuming it still works with the hash map
      std::string name;
      pid_t pid;
      pid_t gpid;
      pid_t sid;
      bool online = false;
      Gtk::Label* status_label;
      Gtk::Label* name_label;
      // std::vector<SubSystemProcess> processes; //! Currently, each subsystem can only run one process (so make it a launch file)
      SubSystemProcess process;
    };
    
    std::vector<std::string> monitored_system_names = {"control_base", "drive_control", "camera_decompressors", "arm_control", "science", "perceptions"};
    // MonitoredSystem control_base_sys;
    std::unordered_map<std::string, MonitoredSystem> monitored_systems;
 
    
    // std::map

    //? Gtk Stuffs
    #include "gtkwidgets.h"
      SubSysStatusGrid system_health;


  //* Button callbacks, either to be triggered by a button in the HMI or a control base panel callback
  void subsystemRequest(std::string subsystem_name, int request);


  //? Ros2 stuffs
 rclcpp::Subscription<rover_msgs::msg::SubSystemHealth>::SharedPtr heartbeat_monitor_sub;
 rclcpp::Publisher<rover_msgs::msg::HeartRequest>::SharedPtr onboard_heart_request_pub;
 rclcpp::Publisher<rover_msgs::msg::HeartRequest>::SharedPtr control_base_heart_request_pub;

  void heartbeatCallback(const rover_msgs::msg::SubSystemHealth::SharedPtr msg);
//  rclcpp::Publishe?


  //? watchdog stuffs
  std::unordered_map<std::string, pid_t> running_child_pids;
    //* CHILD PROCESSES
    void killSubSystem(std::string subsystem_name);
    void runSubSystem(std::string subsystem_name);
    void killProcessGroup(pid_t pgid);
    void runChildNode(std::string pkg, std::string node_or_launch_file, std::string subsytem_name, int type = NODE, bool kill_orphan = true);
    void killChildPID(pid_t target_pid);
    std::vector<pid_t> getPidsByName(const std::string &processName, bool verbose = false);

};