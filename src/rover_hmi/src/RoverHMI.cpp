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
    node->current_middle_card = "full_control_card";
    node->changeCard(node->current_middle_card);

    node->app = app;
    node->run();
    return 0;
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

void MainHMINode::handleArmAbortButtonClick(){ //! abort arm logic
    rover_msgs::msg::ArmCommand vel_msg;
    vel_msg.cmd_type = 'V';
            vel_msg.velocities.resize(6);
    for(int i = 0; i < 6; i++){
            vel_msg.velocities[i] = 0; //just zero velocity for now. in the future firmware should have a specific call
    }
    arm_cmd_pub->publish(vel_msg);
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
    RCLCPP_INFO(this->get_logger(), "meow arm");
    for(int i = 0; i < msg->positions.size(); i++){
        
        this->axis_pos_label[i]->set_label(floatToStringTruncate(msg->positions[i], 2));
    }
   
}

void MainHMINode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
    // lights_status
    RCLCPP_INFO(this->get_logger(), "meow cmdvel");
}

void MainHMINode::changeCard(std::string target_card){
     middle_stack->set_visible_child(target_card.c_str());
                // m_stack.set_visible_child_name(page_name);
            // rclcpp::delay(4);
        // std::this_thread::sleep_for(std::chrono::seconds(1));
        // middle_stack->set_visible_child("full_control_card");
}

std::string MainHMINode::floatToStringTruncate(float value, int decimals) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(decimals) << value;
    return out.str();
}