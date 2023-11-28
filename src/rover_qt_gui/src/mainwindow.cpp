#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), rclcpp::Node("gui_node") {
    ui.setupUi(this);  // This sets up the GUI as designed in Qt Designer
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

    // arm_publisher = this->create_publisher<rover_msgs::msg::ArmCommand>("/arm/command", qos);
    arm_subscriber = this->create_subscription<rover_msgs::msg::ArmCommand>(
    "/arm/feedback", 10, std::bind(&MainWindow::ArmCallback, this, std::placeholders::_1));
    // Connect signals to slots here, for example:
    connect(ui.homeButton, &QPushButton::clicked, this, &MainWindow::onHomeButtonClicked);
}


void MainWindow::onHomeButtonClicked(){
 RCLCPP_INFO(this->get_logger(), "Homing");
 
   
}

void MainWindow::ArmCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg){
    

}