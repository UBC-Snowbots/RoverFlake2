#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), rclcpp::Node("gui_node") {
    ui.setupUi(this);  // This sets up the GUI as designed in Qt Designer
   // auto node = std::make_shared<MainWindow>();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    arm_publisher = this->create_publisher<rover_msgs::msg::ArmCommand>("/arm/command", qos);

    arm_subscriber = this->create_subscription<rover_msgs::msg::ArmCommand>(
    "/arm/feedback", 10, std::bind(&MainWindow::ArmCallback, this, std::placeholders::_1));

    joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&MainWindow::JointStateCallback, this, std::placeholders::_1));

    // Connect signals to slots here, for example:
    connect(ui.homeButton, &QPushButton::clicked, this, &MainWindow::onHomeButtonClicked);
    connect(ui.commButton, &QPushButton::clicked, this, &MainWindow::onCommButtonClicked);

    ui.axis1_pos->setValue(0.0);
    ui.axis2_pos->setValue(0.0);
    ui.axis3_pos->setValue(0.0);
    ui.axis4_pos->setValue(0.0);
    ui.axis5_pos->setValue(0.0);
    ui.axis6_pos->setValue(0.0);
    //MainWindow::spin();
  //  Node::spin();
      //rclcpp::spin(node);
}


void MainWindow::onHomeButtonClicked(){
 RCLCPP_INFO(this->get_logger(), "Homing, if the arm wants to");
 rover_msgs::msg::ArmCommand home_msg;
 home_msg.cmd_type = HOME_CMD;
 arm_publisher->publish(home_msg);
   
}

void MainWindow::onCommButtonClicked(){
 RCLCPP_INFO(this->get_logger(), "Comming");
 rover_msgs::msg::ArmCommand comm_msg;
 comm_msg.cmd_type = COMM_CMD;
 arm_publisher->publish(comm_msg);
   
}

void MainWindow::ArmCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg){
    

}

void MainWindow::JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg){
    ui.axis1_pos->setValue(msg->position[0]);
    ui.axis2_pos->setValue(msg->position[1]);
    ui.axis3_pos->setValue(msg->position[2]);
    ui.axis4_pos->setValue(msg->position[3]);
    ui.axis5_pos->setValue(msg->position[4]);
    ui.axis6_pos->setValue(msg->position[5]);

    RCLCPP_INFO(this->get_logger(), "joints");
    
}