#include "mainwindow.h"

ArmWindow::ArmWindow(QWidget *parent) : QMainWindow(parent), rclcpp::Node("gui_node") {
    ui.setupUi(this);  // This sets up the GUI as designed in Qt Designer

    // Connect signals to slots here, for example:
    connect(ui.homeButton, &QPushButton::clicked, this, &ArmWindow::onHomeButtonClicked);
    connect(ui.commButton, &QPushButton::clicked, this, &ArmWindow::onCommButtonClicked);

}


void ArmWindow::onHomeButtonClicked(){
 RCLCPP_INFO(this->get_logger(), "Homing, if the arm wants to");
 rover_msgs::msg::ArmCommand home_msg;
 home_msg.cmd_type = HOME_CMD;
 //arm_publisher->publish(home_msg);
   
}

void ArmWindow::onCommButtonClicked(){
 RCLCPP_INFO(this->get_logger(), "Comming");
 rover_msgs::msg::ArmCommand comm_msg;
 comm_msg.cmd_type = COMM_CMD;
 arm_publisher->publish(comm_msg);
   
}

void ArmWindow::ArmCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg){
    

}

void ArmWindow::JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "joints");
  
    ui.axis1_pos->setValue(msg->position[0]);
    ui.axis2_pos->setValue(msg->position[1]);
    ui.axis3_pos->setValue(msg->position[2]);
    ui.axis4_pos->setValue(msg->position[3]);
    ui.axis5_pos->setValue(msg->position[4]);
    ui.axis6_pos->setValue(msg->position[5]);

    RCLCPP_INFO(this->get_logger(), "joints");
    
}