#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/arm_command.hpp>
#include <gtkmm.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

class ArmPanelNode : public rclcpp::Node, public Gtk::Window
{
public:
    ArmPanelNode() : Node("arm_panel_node")
    {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("rover_glade_gui");
        
        set_title("ARM PANEL");
        arm_subscriber = this->create_subscription<rover_msgs::msg::ArmCommand>("/arm/command", 10, std::bind(&ArmPanelNode::ArmCommandCallback, this, std::placeholders::_1));
        
        
        
        //*GTKmm stuff
        std::string glade_file_path = package_share_directory + "/glade_files/arm_panel.glade";
        auto builder = Gtk::Builder::create_from_file(glade_file_path.c_str());
        builder->get_widget("arm_panel_top_window", arm_window);
        // builder->get_widget("main_layout", main_layout);  
        // timer_ = this->create_wall_timer(
        //     std::chrono::seconds(1), std::bind(&ArmPanelNode::timer_callback, this));


    }

    ~ArmPanelNode() {
        // Free the allocated memory
        RCLCPP_ERROR(this->get_logger(), "ARM PANEL GOING OFFLINE");
    }

    rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr arm_subscriber;

    // void handle_button_click(GtkButton *button);
    void ArmCommandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg);
    Glib::RefPtr<Gtk::Application> app;
   void run() {
        // Start the GTK main loop
        app->run(*arm_window);
    }
private:
    void timer_callback()
    {
        // auto message = std_msgs::msg::String();
        // message.data = "Hello, world!";
        // publisher_->publish(message);
        
    }

//*GTK objects

    Gtk::Window* arm_window = nullptr;
    // rclcpp::TimerBase::SharedPtr timer_;



};


void ArmPanelNode::ArmCommandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg){
        RCLCPP_ERROR(this->get_logger(), "ARM COMMAND TRIG");

}