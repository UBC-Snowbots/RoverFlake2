#include <RoverHMI.h>


void load_css(const Glib::RefPtr<Gtk::CssProvider>& provider, std::string css_file_path){
    try {
        provider->load_from_path(css_file_path);
    } catch (const Glib::Error& err) {
        // RCLCPP_ERROR(this->get_logger(), "Css load failed: %s", err.what().c_str());
        std::cout << "Css load fail" << std::endl;
    }
}