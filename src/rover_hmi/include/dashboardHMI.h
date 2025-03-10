#include <HMICommon.h>
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
        


        RCLCPP_INFO(this->get_logger(), "BUILDER SUCCESS");

        pid_t pid = fork();
        if (pid == 0) {
            // Child process: replace this process with "ros2 run my_package my_node"
            execlp("ros2", "ros2", "run", "joy", "joy_node", (char *)NULL);
    
            // If execlp() returns, there was an error
            perror("execlp failed");
            exit(EXIT_FAILURE);
    
        } else if (pid < 0) {
            // fork() error in parent
            perror("fork failed");
        } else {
            // Parent process: fork succeeded
            // 'pid' is the child's PID. We could store it if we want to terminate later.
            printf("Launched process with PID: %d\n", pid);
        }
    }

    ~DashboardHMINode(){
        std::system("notify-send DASHBOARD_OFFLINE going down with regular cleanup!");
        rclcpp::shutdown();
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
    



    //? Gtk Stuffs
    Gtk::Window* dash_window;
    Gtk::Layout* dash_layout;


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
//   rclcpp::Subscription heartbeat_monitor_sub;
};