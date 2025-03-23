#include <science_moduleHMI.h>


int main(int argc, char* argv[]){

    int nullc = 0;
    char **nullv = nullptr;
    auto app = Gtk::Application::create(nullc, nullv, "com.example.GtkApplication"); //give GTK null args. ROS launch files add on --ros-args which messes up GTK. 
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<ScienceHMINode>();
    
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



// Set sequence (only one can be active)
void ScienceHMI::setSequence(SequenceType sequence) {
    rover_msgs::msg::ScienceHmi home_msg;

    sequence_status button_triggered = static_cast<sequence_status>(button);
    switch(button_triggered){
        case cmd_vel_buttons::rinse: 
            RCLCPP_INFO(this->get_logger(), "rinse");
            home_msg.sequence_status = 1; 
            break;
    }

        sequence_cmd_pub->publish(home_msg);
        //RCLCPP_INFO(this->get_logger(), "Active Sequence: %s", msg.data.c_str());

        // Reset all sequence buttons
    for (auto button : sequence_buttons) {
        button->get_style_context()->remove_class("active");
    }

    // Set the clicked button as active
    rinsesequence_button->get_style_context()->add_class("active");
}


// Toggle valve states (Multiple can be active)
void ScienceHMI::SV1clicked() {
    rover_msgs::msg::ScienceHmi home_msg;
    home_msg.sv1_status = 1;
    //home_msg.cmd_value = HOME_ALL_ID;
    valve_cmd_pub->publish(home_msg);

    // Get the style context of the button
    auto style_context = sv1_button->get_style_context();

    // Toggle between "active" and "neutral" states
    if (style_context->has_class("active")) {
        style_context->remove_class("active");
        style_context->add_class("neutral");  // Change to neutral (yellow)
    } else if (style_context->has_class("neutral")) {
        style_context->remove_class("neutral");
        style_context->add_class("active");  // Change back to active (green)
    } else {
        style_context->add_class("active");  // Default to active if neither is set
    }
}

void ScienceHMINode::handleTextboxInput() {
    std::string input_text = valve_input_textbox->get_text();
    try {
        int index = std::stoi(input_text);
        setCarouselIndex(index);
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Invalid input for carousel index.");
    }
}



// Set carousel index (0-15)
void ScienceHMI::setCarouselIndex(int index) {
    rover_msgs::msg::ScienceHmi home_msg;
    
    if (index >= 0 && index <= 15) {
        home_msg.carousel_index = index;
        carousel_cmd_pub->publish(home_msg);
        RCLCPP_INFO(this->get_logger(), "Carousel Index Set: %d", index);
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid Carousel Index! Must be between 0-15.");
    }
}
