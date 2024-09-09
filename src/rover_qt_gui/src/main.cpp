#include <QTimer>
#include <QApplication>
#include "mainwindow.h"


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);  // Initialize ROS2
    QApplication app(argc, argv);  // Initialize Qt
    auto node = std::make_shared<ArmWindow>();
    node->init();
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [node]() {
        rclcpp::spin_some(node);
    });
    timer.start(20); // Set the interval to 20 ms
    
    ArmWindow window;
    window.show();

    return app.exec(); // Start the Qt event loop; 
}



