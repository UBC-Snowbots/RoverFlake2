#include <RoverHMI.h>


int main(int argc, char* argv[]){

    int nullc = 0;
    char **nullv = nullptr;
    auto app = Gtk::Application::create(nullc, nullv, "com.example.GtkApplication"); //give GTK null args. ROS launch files add on --ros-args which messes up GTK. 
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<MainHMINode>();
    
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
    // node->current_middle_card = "full_control_card";
    // node->changeCard(node->available_cards[0]);

    node->app = app;
    node->run();
    return 0;
}

//* @brief Switches cards
//* @param bool next, if true switches to next card, if false switches to previous card
//* If at end of cards, rollback to first card
void MainHMINode::handleCardButtonSwitch(bool next){
    // cards target_card;
    
    RCLCPP_INFO(this->get_logger(), "Card Switch Detected");
    if(next){
        //? Switch to next card
        current_card_i++;

        if(current_card_i >= static_cast<int>(cards::num_cards)){ 
        current_card_i = 0;
        }
    }else{
        //? Switch to previous card
        current_card_i--;
    if(current_card_i < 0){
        current_card_i = static_cast<int>(cards::num_cards) - 1;
    }
    
    }
    
    changeCard(available_cards[current_card_i]); 
}


bool MainHMINode::handleVideoFrameDraw(const Cairo::RefPtr<Cairo::Context>& cr)
{
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (pixbuf_)
    {
        Gdk::Cairo::set_source_pixbuf(cr, pixbuf_, 0, 0);
        cr->paint();
    }
    return true;
}

void MainHMINode::handleCmdVelButton(bool pressed, int button){
    RCLCPP_INFO(this->get_logger(), "CMD VEL BUTTON pressed: %d button: %d", pressed, button);
    float linear = 0;
    float angular = 0;
    geometry_msgs::msg::Twist msg;
    // float linscale
    cmd_vel_buttons button_triggered = static_cast<cmd_vel_buttons>(button);
    switch(button_triggered){
        case cmd_vel_buttons::forward: //* Full speed ahead!
            RCLCPP_INFO(this->get_logger(), "forward");
            linear = linear_magslider->get_value() * 1; 
            angular = 0;
            break;
        case cmd_vel_buttons::reverse: //* Full speed backwards!
            RCLCPP_INFO(this->get_logger(), "reverse");
            linear = linear_magslider->get_value() * -1; 
            angular = 0;
            break;
        case cmd_vel_buttons::right_forward: //* Full speed ahead! Half speed right!
            RCLCPP_INFO(this->get_logger(), "right_forward");
            linear = linear_magslider->get_value() * 1;
            angular = angular_magslider->get_value() * 0.5;
            break;
        case cmd_vel_buttons::left_forward: //* ya get the point
            RCLCPP_INFO(this->get_logger(), "left_forward");
            linear = linear_magslider->get_value() * 1;
            angular = angular_magslider->get_value() * -0.5;
            break;
        case cmd_vel_buttons::left_reverse:
            RCLCPP_INFO(this->get_logger(), "left_reverse");
            linear = linear_magslider->get_value() * -1;
            angular = angular_magslider->get_value() * -0.5;
            break;
        case cmd_vel_buttons::right_reverse:
            RCLCPP_INFO(this->get_logger(), "right_reverse");
            linear = linear_magslider->get_value() *-1;
            angular = angular_magslider->get_value() * 0.5;
            break;
        case cmd_vel_buttons::rotate_cw:
            RCLCPP_INFO(this->get_logger(), "rotate_cw");
            linear = 0;
            angular = angular_magslider->get_value() * 1;
            break;
        case cmd_vel_buttons::rotate_ccw:
            RCLCPP_INFO(this->get_logger(), "rotate_ccw");
            angular = angular_magslider->get_value() * -1;
            linear = 0;
            break;
    }
    if(!pressed){
        linear = 0;
        angular = 0;
    }
    
    RCLCPP_INFO(this->get_logger(), "Linear: %f, Angular: %f", linear, angular);
    msg.linear.x = linear;
    msg.angular.z = angular;
    cmd_vel_pub->publish(msg);

}

void MainHMINode::handleHomeAllButtonClick(){
    rover_msgs::msg::ArmCommand home_msg;
    home_msg.cmd_type = 'h';
    home_msg.cmd_value = HOME_ALL_ID;
    arm_cmd_pub->publish(home_msg);
    RCLCPP_WARN(this->get_logger(), "Home all button clicked!");
    // RCLCPP_WARN(this->get_logger(), "NOTHING SENT!");

}

void MainHMINode::handleIncAxisButtonClick(int index){
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



void MainHMINode::handleAxisButtonRelease(){
    rover_msgs::msg::ArmCommand vel_msg;
    vel_msg.cmd_type = ABS_VEL_CMD;
    vel_msg.velocities.resize(6);
    for(int i = 0; i < 6; i++){
        vel_msg.velocities[i] = 0;
    }
    arm_cmd_pub->publish(vel_msg);

}

void MainHMINode::ik_timer_callback(){
     geometry_msgs::msg::TwistStamped ik_msg;

    ik_msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    ik_msg.header.frame_id = "link_tt";
    int index = current_ik_index;
    float value = ik_hmi_speed[index]; ///TODO get speed based on spinbuttons
    switch (index)
    {
        case 0: // Linear X
            ik_msg.twist.linear.x = value;
            break;
        case 1: // Linear Y
            ik_msg.twist.linear.y = value;
            break;
        case 2: // Linear Z
            ik_msg.twist.linear.z = value;
            break;
        case 3: // Angular X
            ik_msg.twist.angular.x = value;
            break;
        case 4: // Angular Y
            ik_msg.twist.angular.y = value;
            break;
        case 5: // Angular Z
            ik_msg.twist.angular.z = value;
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Invalid index: %d. Must be 0-5.", index);
            return;
    }
    
    arm_ik_pub->publish(ik_msg);
}

void MainHMINode::handleIncIKButtonClick(int index){
    current_ik_index = index;
    ik_hmi_speed[index] = abs(ik_hmi_speed[index]);
    ik_timer->reset();
    
}
void MainHMINode::handleDecIKButtonClick(int index){
    current_ik_index = index;
    // current_ik_value = abs(current_ik_value)*-1;
    ik_hmi_speed[index] = abs(ik_hmi_speed[index])*-1;
    ik_timer->reset();
    // geometry_msgs::msg::TwistStamped ik_msg;
    // ik_msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    // ik_msg.header.frame_id = "link_tt";
    // float value = -0.5; ///TODO get speed based on spinbuttons
    // switch (index)
    // {
    //     case 0: // Linear X
    //         ik_msg.twist.linear.x = value;
    //         break;
    //     case 1: // Linear Y
    //         ik_msg.twist.linear.y = value;
    //         break;
    //     case 2: // Linear Z
    //         ik_msg.twist.linear.z = value;
    //         break;
    //     case 3: // Angular X
    //         ik_msg.twist.angular.x = value;
    //         break;
    //     case 4: // Angular Y
    //         ik_msg.twist.angular.y = value;
    //         break;
    //     case 5: // Angular Z
    //         ik_msg.twist.angular.z = value;
    //         break;
    //     default:
    //         RCLCPP_WARN(this->get_logger(), "Invalid index: %d. Must be 0-5.", index);
    //         return;
    // }
    
    // arm_ik_pub->publish(ik_msg);
    
}



void MainHMINode::handleIKButtonRelease(){
    ik_timer->cancel();
    geometry_msgs::msg::TwistStamped ik_msg;
    ik_msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    ik_msg.header.frame_id = "link_tt";
    ik_msg.twist.linear.x = 0;
    ik_msg.twist.linear.y = 0;
    ik_msg.twist.linear.z = 0;
    ik_msg.twist.angular.x = 0;
    ik_msg.twist.angular.y = 0;
    ik_msg.twist.angular.z = 0;

    
    arm_ik_pub->publish(ik_msg);
}

void MainHMINode::handleDecEEButtonClick(){
    rover_msgs::msg::ArmCommand vel_msg;
    vel_msg.cmd_type = 'V';
        vel_msg.velocities.resize(6);
    for(int i = 0; i < 6; i++){
            vel_msg.velocities[i] = 0;
    }
    vel_msg.end_effector = ee_speed * -1;
    arm_cmd_pub->publish(vel_msg);
}
void MainHMINode::handleIncEEButtonClick(){
    rover_msgs::msg::ArmCommand vel_msg;
    vel_msg.cmd_type = 'V';
        vel_msg.velocities.resize(6);
    for(int i = 0; i < 6; i++){
            vel_msg.velocities[i] = 0;
    }
    vel_msg.end_effector = ee_speed;
    arm_cmd_pub->publish(vel_msg);
}



void MainHMINode::handleDecAxisButtonClick(int index){
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


void MainHMINode::handleAxisSpeedUpdate(int i) {
  
        double new_value = axis_speed_spinbutton[i]->get_value();
        axis_hmi_speed[i] = new_value;
        RCLCPP_INFO(this->get_logger(), "Axis %i speed changed to %f", i, new_value);
    
}

void MainHMINode::handleIKSpeedUpdate(int i) {
  
        double new_value = ik_speed_spinbutton[i]->get_value();
        ik_hmi_speed[i] = new_value;
        RCLCPP_INFO(this->get_logger(), "IK %i speed changed to %f", i, new_value);
    
}

void MainHMINode::handleArmAbortButtonClick(){ //! abort arm logic
    rover_msgs::msg::ArmCommand vel_msg;
    vel_msg.cmd_type = 'V';
            vel_msg.velocities.resize(6);
    for(int i = 0; i < 6; i++){
            vel_msg.velocities[i] = 0; //just zero velocity for now. in the future firmware should have a specific call if we need like an abort sequence
    }
    vel_msg.end_effector = 0;
    arm_cmd_pub->publish(vel_msg);
    handleIKButtonRelease();

    RCLCPP_ERROR(this->get_logger(), "Arm Movements Abort Demanded.");
}

void MainHMINode::handlePosFeedOnButtonClick(){
    rover_msgs::msg::ArmCommand comm_on_msg;
    comm_on_msg.cmd_type = 'C';
    comm_on_msg.cmd_value = 1;
    arm_cmd_pub->publish(comm_on_msg);
    RCLCPP_WARN(this->get_logger(), "Position Feed On button clicked!");

}
void MainHMINode::handlePosFeedOffButtonClick(){
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = 'C';
    msg.cmd_value = 0;
    arm_cmd_pub->publish(msg);
    RCLCPP_WARN(this->get_logger(), "Position Feed Off Button clicked!");

}

void MainHMINode::handleTestLimitsButtonClick(){
    rover_msgs::msg::ArmCommand msg;
    msg.cmd_type = 't';
    // msg.cmd_value = 0;
    arm_cmd_pub->publish(msg);
    RCLCPP_WARN(this->get_logger(), "Test limits Button clicked!");

}

void MainHMINode::image_feed_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(image_mutex_);
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        image_ = cv_ptr->image;

        // Convert OpenCV image to Gdk::Pixbuf
        pixbuf_ = Gdk::Pixbuf::create_from_data(
            image_.data, Gdk::COLORSPACE_RGB, false, 8, image_.cols, image_.rows, image_.step);

        // Queue a redraw event
        image_draw_area->queue_draw();
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
}


void MainHMINode::armFeedbackCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg){
    // RCLCPP_INFO(this->get_logger(), "meow arm");
    for(int i = 0; i < msg->positions.size(); i++){
        
        this->axis_pos_label[i]->set_label(floatToStringTruncate(msg->positions[i], 2));
    }
   
}

void MainHMINode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
    // lights_status
    // RCLCPP_INFO(this->get_logger(), "meow cmdvel");
}

// void MainHMINode::changeCard(cards target_card){
//     std::string target_card_name = available_cards[static_cast<int>(target_card)];
//      middle_stack->set_visible_child(target_card_name.c_str());
// }
void MainHMINode::changeCard(std::string target_card){
     middle_stack->set_visible_child(target_card.c_str());
}

std::string MainHMINode::floatToStringTruncate(float value, int decimals) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(decimals) << value;
    return out.str();
}