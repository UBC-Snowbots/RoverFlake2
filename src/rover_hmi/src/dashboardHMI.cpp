#include <dashboardHMI.h>


int main(int argc, char* argv[]){
    //Gtk is a picky eater and crashes when you feed it ros args, which are automatically fed with launch file, and i think regular running.
    // So, GTK gets null args. 
    int nullc = 0;
    char **nullv = nullptr;

    auto app = Gtk::Application::create(nullc, nullv, "com.example.GtkApplication");
    //CHAD ROS2 gets the real arguments from the terminal
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DashboardHMINode>();

    Glib::signal_timeout().connect([&node]() -> bool {
        rclcpp::spin_some(node);
        return true;
    }, 20); //MILlISECINDS

    node->app = app;
    node->run();
    return 0;


}