#include <RoverHMI.h>


int main(int argc, char* argv[]){
    int nullc = 0;
    char **nullv = nullptr;
    auto app = Gtk::Application::create(nullc, nullv, "com.example.GtkApplication"); //give GTK null args. ROS launch files add on --ros-args which messes up GTK. 
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<MainHMINode>();
    
    if(*argv == "hi"){
        RCLCPP_INFO(node->get_logger(), "hi");
    }

    node->app = app;

    Glib::signal_timeout().connect([&node]() -> bool){
        rclcpp::spin_some(node);
        return true;
    }, 1);


    node->run();
    return 0;
}