#include <QApplication>
#include "mainwindow.h"


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);  // Initialize ROS2
    QApplication app(argc, argv);  // Initialize Qt
    MainWindow window;
    auto node = std::make_shared<MainWindow>();
    window.show();
    app.exec();
    rclcpp::spin(node);
    return 0;  // Start the Qt event loop
}



