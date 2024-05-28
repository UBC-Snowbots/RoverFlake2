#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/arm_command.hpp>
#include <gtkmm.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

class MainDisplayLeftNode : public rclcpp::Node, public Gtk::Window
{
public:
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("rover_glade_gui");

    MainDisplayLeftNode() : Node("main_display_left_node"){
        //set_title()

        RCLCPP_INFO(this->get_logger(), "MAIN DISPLAY LEFT INIT");

    }

    void init(){
        //constructor should handle everything?

        RCLCPP_INFO(this->get_logger(), "MAIN DISPLAY LEFT READY");
    }

//deconstructor
~MainDisplayLeftNode(){
    RCLCPP_ERROR(this->get_logger(), "MAIN DISPLAY LEFT GOING OFFLINE");
}

Glib::RefPtr<Gtk::Application> app;
void run() {
    // Start the GTK main loop
    app->run(*main_display_left_window);
}

private:
    //timer would go here
    //can ROS spin with GTK??


Gtk::Window* main_display_left_window = nullptr;

};