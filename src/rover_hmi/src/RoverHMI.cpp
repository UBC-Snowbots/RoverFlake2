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

    Glib::signal_timeout().connect([&node]() -> bool {
        if(node->current_middle_card == "full_control_card"){
            node->current_middle_card = "system_overview_card";
            node->changeCard(node->current_middle_card);
        }else{
            node->current_middle_card = "full_control_card";
            node->changeCard(node->current_middle_card);
        }
        return true;
    }, 2000);

    node->app = app;
    node->run();
    return 0;
}



void MainHMINode::armFeedbackCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "meow arm");
}

void MainHMINode::changeCard(std::string target_card){
     middle_stack->set_visible_child(target_card.c_str());
                // m_stack.set_visible_child_name(page_name);
            // rclcpp::delay(4);
        // std::this_thread::sleep_for(std::chrono::seconds(1));
        // middle_stack->set_visible_child("full_control_card");
}