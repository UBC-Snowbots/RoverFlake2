#include <HMICommon.h>
#include <rover_msgs/msg/sub_system_health.hpp>
#define NUM_MONITORED_SYSTEMS 6
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

        //* Glade builder setup
        builder->get_widget("dash_window", dash_window);
        builder->get_widget("dash_top_layout", dash_layout);
            //* subsys Status Grid
            builder->get_widget("subsys_status_grid", system_health.grid);
                builder->get_widget("subsys_0_label", system_health.system[0].name);
                builder->get_widget("subsys_1_label", system_health.system[1].name);
                builder->get_widget("subsys_2_label", system_health.system[2].name);
                builder->get_widget("subsys_3_label", system_health.system[3].name);
                builder->get_widget("subsys_4_label", system_health.system[4].name);
                builder->get_widget("subsys_5_label", system_health.system[5].name);
                builder->get_widget("subsys_0_status_label", system_health.system[0].status);
                builder->get_widget("subsys_1_status_label", system_health.system[1].status);
                builder->get_widget("subsys_2_status_label", system_health.system[2].status);
                builder->get_widget("subsys_3_status_label", system_health.system[3].status);
                builder->get_widget("subsys_4_status_label", system_health.system[4].status);
                builder->get_widget("subsys_5_status_label", system_health.system[5].status);

            

                for(int i = 0; i < NUM_MONITORED_SYSTEMS; i++){
                    system_health.system[i].name->set_label(monitored_systems[i]);
                    system_health.system[i].status->set_label("UNKNOWN");
                }
        
      builder->get_widget("global_msg_box", global_msg_label);
                global_msg_label->set_label("DASHBOARD STARTING...");

        RCLCPP_INFO(this->get_logger(), "BUILDER SUCCESS");
        global_msg_label->set_label("DASHBOARD STARTED");

    }

    ~DashboardHMINode(){
        std::system("notify-send DASHBOARD_OFFLINE going down with regular cleanup!");
        //! need to add orphan cleanup here
        for(const auto &pair : running_child_pids){
          RCLCPP_INFO(this->get_logger(), "Performing standard execution of child: %s, with pid %d", pair.first, pair.second);
          // killChildPID(pair.second);
          kill(pair.second, SIGTERM); 
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for children to say their last words...");
        sleep(5);
        RCLCPP_INFO(this->get_logger(), "Times up, forcefully killing any slow orphans");
        for(const auto &pair : running_child_pids){
          RCLCPP_INFO(this->get_logger(), "Performing standard execution of child: %s, with pid %d", pair.first, pair.second);
          // killChildPID(pair.second);
          kill(pair.second, SIGKILL);
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
    std::vector<std::string> monitored_systems = {"cbs_daemon", "drive_control", "arm_hardware", "arm_control", "science", "perceptions"};
    // std::vector<std::string> running_childs;
    
    // std::map

    //? Gtk Stuffs
    Gtk::Window* dash_window;
    Gtk::Layout* dash_layout;

      //* Global Messaging System
      Gtk::Label* global_msg_label;


  struct SubSysStatusElement
  {
    Gtk::Label* name;
    Gtk::Label* status;

  };
     //* System Overview
  struct SubSysStatusGrid
  {
    Gtk::Grid* grid;
    SubSysStatusElement system[NUM_MONITORED_SYSTEMS];
  };


  SubSysStatusGrid system_health;


  //? Ros2 stuffs
 rclcpp::Subscription<rover_msgs::msg::SubSystemHealth>::SharedPtr heartbeat_monitor_sub;
  void heartbeatCallback(const rover_msgs::msg::SubSystemHealth::SharedPtr msg);
//  rclcpp::Publishe?



  //? watchdog stuffs
  std::unordered_map<std::string, pid_t> running_child_pids;
    //* CHILD PROCESSES
    void runChildNode(std::string pkg, std::string node_or_launch_file, bool launch, bool kill_orphan);
    void killChildPID(pid_t target_pid);
};