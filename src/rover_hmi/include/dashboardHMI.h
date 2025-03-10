#include <HMICommon.h>
#define NUM_MONITORED_NODES 5
// #include <helper_functions.h>

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
            //* Node Status Grid
            builder->get_widget("node_status_grid", node_stats.grid);
                builder->get_widget("node_0_label", node_stats.nodes[0].name);
                builder->get_widget("node_1_label", node_stats.nodes[1].name);
                builder->get_widget("node_0_status_label", node_stats.nodes[0].status);
                builder->get_widget("node_1_status_label", node_stats.nodes[1].status);

                node_stats.nodes[0].name->set_label(monitored_nodes[0]);
                node_stats.nodes[1].name->set_label(monitored_nodes[1]);

                for(int i = 0; i < 2; i++){
                    node_stats.nodes[i].name->set_label(monitored_nodes[i]);
                    node_stats.nodes[i].status->set_label("UNKNOWN");
                }
        


        RCLCPP_INFO(this->get_logger(), "BUILDER SUCCESS");
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
    std::vector<std::string> monitored_nodes = {"cbs_daemon", "drive_control"};
    



    //? Gtk Stuffs
    Gtk::Window* dash_window;
    Gtk::Layout* dash_layout;


  struct NodeStatusElement
  {
    Gtk::Label* name;
    Gtk::Label* status;

  };
     //* System Overview
  struct NodeStatusGrid
  {
    Gtk::Grid* grid;
    NodeStatusElement nodes[NUM_MONITORED_NODES];
  };


  NodeStatusGrid node_stats;


  //? Ros2 stuffs
//   rclcpp::Subscription heartbeat_monitor_sub;
};