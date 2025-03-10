#pragma once
//file for all common inclusions.

#include <gtkmm.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>


#include <iostream>
#include <sstream>
#include <iomanip>  // For std::setprecision

// Global functions
void load_css(const Glib::RefPtr<Gtk::CssProvider>& provider, std::string css_file_path);
