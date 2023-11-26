#include <QApplication>
#include "mainwindow.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);  // Initialize ROS2
    QApplication app(argc, argv);  // Initialize Qt

    MainWindow window;
    window.show();

    return app.exec();  // Start the Qt event loop
}