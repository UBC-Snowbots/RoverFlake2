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