#include <ArmTestingHMI.h>


int main(int argc, char* argv[]){

    int nullc = 0;
    char **nullv = nullptr;
    auto app = Gtk::Application::create(nullc, nullv, "com.example.GtkApplication"); //give GTK null args. ROS launch files add on --ros-args which messes up GTK. 
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<ArmHMINode>();
    
    // if(argv == "hi"){
    //     RCLCPP_INFO(node->get_logger(), "hi");
    // }

    Glib::signal_timeout().connect([&node]() -> bool {
        rclcpp::spin_some(node);
        return true;
    }, 20);

    // Glib::signal_timeout().connect([&node]() -> bool {
    //     if(node->current_middle_card == "full_control_card"){
    //         // node->current_middle_card = "system_overview_card";
    //         node->changeCard(node->current_middle_card);
    //     }else if(node->current_{
    //         node->current_middle_card = "full_control_card";
    //         node->changeCard(node->current_middle_card);
    //     }
    //     return true;
    // }, 2000);
    // node->changeCard(node->current_middle_card);

    node->app = app;
    node->run();
    return 0;
}

// bool ArmHMINode::handleVideoFrameDraw(const Cairo::RefPtr<Cairo::Context>& cr)
// {
//     std::lock_guard<std::mutex> lock(image_mutex_);
//     if (pixbuf_)
//     {
//         Gdk::Cairo::set_source_pixbuf(cr, pixbuf_, 0, 0);
//         cr->paint();
//     }
//     return true;
// }

void ArmHMINode::handleHomeAllButtonClick(){
    rover_msgs::msg::ArmCommand home_msg;
    home_msg.cmd_type = 'h';
    home_msg.cmd_value = HOME_ALL_ID;
    arm_cmd_pub->publish(home_msg);
    RCLCPP_WARN(this->get_logger(), "Home all button clicked!");
    // RCLCPP_WARN(this->get_logger(), "NOTHING SENT!");

}

void ArmHMINode::handleIncAxisButtonClick(int index){
    rover_msgs::msg::ArmCommand vel_msg;
    vel_msg.cmd_type = ABS_VEL_CMD;
    vel_msg.velocities.resize(6);
    for(int i = 0; i < 6; i++){
        if(index != i){
            vel_msg.velocities[i] = 0;
        }else{
            vel_msg.velocities[i] = axis_hmi_speed[i];

        }

    }
    arm_cmd_pub->publish(vel_msg);
    
}

void ArmHMINode::handleAxisButtonRelease(){
    rover_msgs::msg::ArmCommand vel_msg;
    vel_msg.cmd_type = ABS_VEL_CMD;
    vel_msg.velocities.resize(6);
    for(int i = 0; i < 6; i++){
        vel_msg.velocities[i] = 0;
    }
    arm_cmd_pub->publish(vel_msg);

}



void ArmHMINode::handleDecAxisButtonClick(int index){
    rover_msgs::msg::ArmCommand vel_msg;
    vel_msg.cmd_type = 'V';
        vel_msg.velocities.resize(6);
    for(int i = 0; i < 6; i++){
        if(index != i){
            vel_msg.velocities[i] = 0;
        }else{
            vel_msg.velocities[i] = -1 * axis_hmi_speed[i];

        }

    }
    arm_cmd_pub->publish(vel_msg);
}


void ArmHMINode::handleAxisSpeedUpdate(int i) {
  
        double new_value = axis_speed_spinbutton[i]->get_value();
        axis_hmi_speed[i] = new_value;
        RCLCPP_INFO(this->get_logger(), "Axis %i speed changed to %f", i, new_value);
    
}

void ArmHMINode::handleArmAbortButtonClick(){ //! abort arm logic
    rover_msgs::msg::ArmCommand vel_msg;
    vel_msg.cmd_type = 'V';
            vel_msg.velocities.resize(6);
    for(int i = 0; i < 6; i++){
            vel_msg.velocities[i] = 0; //just zero velocity for now. in the future firmware should have a specific call
    }
    arm_cmd_pub->publish(vel_msg);
    RCLCPP_ERROR(this->get_logger(), "Arm Movements Abort Demanded.");
}

void ArmHMINode::handlePosFeedOnButtonClick(){
    rover_msgs::msg::ArmCommand comm_on_msg;
    comm_on_msg.cmd_type = 'C';
    comm_on_msg.cmd_value = 1;
    arm_cmd_pub->publish(comm_on_msg);
    RCLCPP_WARN(this->get_logger(), "Position Feed On button clicked!");

}
void ArmHMINode::handlePosFeedOffButtonClick(){
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = 'C';
    msg.cmd_value = 0;
    arm_cmd_pub->publish(msg);
    RCLCPP_WARN(this->get_logger(), "Position Feed Off Button clicked!");

}

void ArmHMINode::handleTestLimitsButtonClick(){
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = 't';
    // msg.cmd_value = 0;
    arm_cmd_pub->publish(msg);
    RCLCPP_WARN(this->get_logger(), "Test limits Button clicked!");

}


void ArmHMINode::armFeedbackCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "meow arm");
    for(int i = 0; i < msg->positions.size(); i++){
        
        this->axis_pos_label[i]->set_label(floatToStringTruncate(msg->positions[i], 2));
    }
   
}

void ArmHMINode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
    // lights_status
    RCLCPP_INFO(this->get_logger(), "meow cmdvel");
}

std::string ArmHMINode::floatToStringTruncate(float value, int decimals) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(decimals) << value;
    return out.str();
}