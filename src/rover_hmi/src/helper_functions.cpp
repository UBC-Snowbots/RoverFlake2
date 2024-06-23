#include <RoverHMI.h>


void MainHMINode::load_css(const Glib::RefPtr<Gtk::CssProvider>& provider){
    try {
        provider->load_from_path(main_css_file_path);
    } catch (const Glib::Error& err) {
        RCLCPP_ERROR(this->get_logger(), "Css load failed: %s", err.what().c_str());
    }
}