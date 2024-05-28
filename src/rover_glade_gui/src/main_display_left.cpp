#include <main_display_left.h>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto app = Gtk::Application::create(argc, argv, "com.example.GtkAppliation");

    auto node = std::make_shared<MainDisplayLeftNode>();
    node->app = app;



//TODO
    node->init();
    // rclcpp::timerbase() timer;
    node->run();
    // rclcpp::spin(node); //!wont run
    rclcpp::shutdown();

    return 0;
}

