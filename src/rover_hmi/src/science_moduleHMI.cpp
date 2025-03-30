#include <science_moduleHMI.h>


int main(int argc, char* argv[]){

    int nullc = 0;
    char **nullv = nullptr;
    auto app = Gtk::Application::create(nullc, nullv, "com.example.ScienceHMI"); //give GTK null args. ROS launch files add on --ros-args which messes up GTK. 
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
void ScienceHMINode::setSequence(bool pressed, int button) {
    rover_msgs::msg::ScienceModule home_msg;

    sequence_status button_triggered = static_cast<sequence_status>(button);
    switch(button_triggered){
        case sequence_status::rinse: 
            RCLCPP_INFO(this->get_logger(), "rinse");
            home_msg.sequenceselection = 1; 
            rinsebutton->get_style_context()->add_class("active");
            break;
        case sequence_status::agitator: 
            RCLCPP_INFO(this->get_logger(), "agitator");
            home_msg.sequenceselection = 2; 
            agitatorbutton->get_style_context()->add_class("active");
            break;
        case sequence_status::process: 
            RCLCPP_INFO(this->get_logger(), "process");
            home_msg.sequenceselection = 3; 
            processbutton->get_style_context()->add_class("active");
            break;
        case sequence_status::purge: 
            RCLCPP_INFO(this->get_logger(), "purge");
            home_msg.sequenceselection = 4; 
            purgebutton->get_style_context()->add_class("active");
            break;
    }

        sequence_pub->publish(home_msg);
        //RCLCPP_INFO(this->get_logger(), "Active Sequence: %s", msg.data.c_str());

        // Reset all sequence buttons
    for (auto button : sequence_buttons) {
        button->get_style_context()->remove_class("active");
    }

    // Set the clicked button as active
}


// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::SV1clicked() {
    rover_msgs::msg::ScienceModule home_msg;
    home_msg.sv1status = 1;
    valve_pub->publish(home_msg);

    // Get the style context of the button
    auto style_context = sv1button->get_style_context();

    // Toggle between "energized" (green) and "not_energized" (red) states
    if (style_context->has_class("energized")) {
        style_context->remove_class("energized");
        style_context->add_class("not_energized");  // Change to not energized (red)
    } else {
        style_context->remove_class("not_energized");
        style_context->add_class("energized");  // Change to energized (green)
    }
}


// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::SV2clicked() {
    rover_msgs::msg::ScienceModule home_msg;
    home_msg.sv2status = 1;
    valve_pub->publish(home_msg);

    // Get the style context of the button
    auto style_context = sv2button->get_style_context();

    // Toggle between "energized" (green) and "not_energized" (red) states
    if (style_context->has_class("energized")) {
        style_context->remove_class("energized");
        style_context->add_class("not_energized");  // Change to not energized (red)
    } else {
        style_context->remove_class("not_energized");
        style_context->add_class("energized");  // Change to energized (green)
    }
}


// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::SV3clicked() {
    rover_msgs::msg::ScienceModule home_msg;
    home_msg.sv3status = 1;
    valve_pub->publish(home_msg);

    // Get the style context of the button
    auto style_context = sv3button->get_style_context();

    // Toggle between "energized" (green) and "not_energized" (red) states
    if (style_context->has_class("energized")) {
        style_context->remove_class("energized");
        style_context->add_class("not_energized");  // Change to not energized (red)
    } else {
        style_context->remove_class("not_energized");
        style_context->add_class("energized");  // Change to energized (green)
    }
}


// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::SV4clicked() {
    rover_msgs::msg::ScienceModule home_msg;
    home_msg.sv4status = 1;
    valve_pub->publish(home_msg);

    // Get the style context of the button
    auto style_context = sv4button->get_style_context();

    // Toggle between "energized" (green) and "not_energized" (red) states
    if (style_context->has_class("energized")) {
        style_context->remove_class("energized");
        style_context->add_class("not_energized");  // Change to not energized (red)
    } else {
        style_context->remove_class("not_energized");
        style_context->add_class("energized");  // Change to energized (green)
    }
}


// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::SVF1clicked() {
    rover_msgs::msg::ScienceModule home_msg;
    home_msg.sv1status = 1;
    valve_pub->publish(home_msg);

    // Get the style context of the button
    auto style_context = svf1button->get_style_context();

    // Toggle between "energized" (green) and "not_energized" (red) states
    if (style_context->has_class("energized")) {
        style_context->remove_class("energized");
        style_context->add_class("not_energized");  // Change to not energized (red)
    } else {
        style_context->remove_class("not_energized");
        style_context->add_class("energized");  // Change to energized (green)
    }
}


// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::SVF2clicked() {
    rover_msgs::msg::ScienceModule home_msg;
    home_msg.svf2status = 1;
    valve_pub->publish(home_msg);

    // Get the style context of the button
    auto style_context = svf2button->get_style_context();

    // Toggle between "energized" (green) and "not_energized" (red) states
    if (style_context->has_class("energized")) {
        style_context->remove_class("energized");
        style_context->add_class("not_energized");  // Change to not energized (red)
    } else {
        style_context->remove_class("not_energized");
        style_context->add_class("energized");  // Change to energized (green)
    }
}

// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::P1clicked() {
    rover_msgs::msg::ScienceModule home_msg;
    home_msg.p1status = 1;
    osf_pub->publish(home_msg);

    // Get the style context of the button
    auto style_context = osf1button->get_style_context();

    // Toggle between "energized" (green) and "not_energized" (red) states
    if (style_context->has_class("forward")) {
        style_context->remove_class("forward");
        style_context->add_class("backward");  // Change to not energized (red)
    } else {
        style_context->remove_class("backward");
        style_context->add_class("forward");  // Change to energized (green)
    }
}

// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::OSF1clicked() {
    rover_msgs::msg::ScienceModule home_msg;
    home_msg.osf1status = 1;
    osf_pub->publish(home_msg);

    // Get the style context of the button
    auto style_context = osf2button->get_style_context();

    // Toggle between "energized" (green) and "not_energized" (red) states
    if (style_context->has_class("unblocked")) {
        style_context->remove_class("unblocked");
        style_context->add_class("blocked");  // Change to not energized (red)
    } else {
        style_context->remove_class("blocked");
        style_context->add_class("unblocked");  // Change to energized (green)
    }
}

// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::OSF2clicked() {
    rover_msgs::msg::ScienceModule home_msg;
    home_msg.osf2status = 1;
    pump_pub->publish(home_msg);

    // Get the style context of the button
    auto style_context = p1button->get_style_context();

    // Toggle between "energized" (green) and "not_energized" (red) states
     if (style_context->has_class("unblocked")) {
        style_context->remove_class("unblocked");
        style_context->add_class("blocked");  // Change to not energized (red)
    } else {
        style_context->remove_class("blocked");
        style_context->add_class("unblocked");  // Change to energized (green)
    }
}


// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::prevClicked() {
    rover_msgs::msg::ScienceModule home_msg;
    home_msg.prevstatus = 1;
    carousel_pub->publish(home_msg);

    // Get the style context of the button
    auto style_context = prevIndexbutton->get_style_context();

    // Toggle between "energized" (green) and "not_energized" (red) states
     if (style_context->has_class("on")) {
        style_context->remove_class("on");
        style_context->add_class("off");  // Change to not energized (red)
    } else {
        style_context->remove_class("off");
        style_context->add_class("on");  // Change to energized (green)
    }
}

// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::nextClicked() {
    rover_msgs::msg::ScienceModule home_msg;
    home_msg.nextstatus = 1;
    carousel_pub->publish(home_msg);

    // Get the style context of the button
    auto style_context = nextidxbutton->get_style_context();

    // Toggle between "energized" (green) and "not_energized" (red) states
    if (style_context->has_class("on")) {
        style_context->remove_class("on");
        style_context->add_class("off");  // Change to not energized (red)
    } else {
        style_context->remove_class("off");
        style_context->add_class("on");  // Change to energized (green)
    }
}

// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::largeClicked() {
    rover_msgs::msg::ScienceModule home_msg;
    home_msg.largestatus = 1;
    carousel_pub->publish(home_msg);

    // Get the style context of the button
    auto style_context = largebutton->get_style_context();

    // Toggle between "energized" (green) and "not_energized" (red) states
    if (style_context->has_class("on")) {
        style_context->remove_class("on");
        style_context->add_class("off");  // Change to not energized (red)
    } else {
        style_context->remove_class("off");
        style_context->add_class("on");  // Change to energized (green)
    }
}

// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::smallClicked() {
    rover_msgs::msg::ScienceModule home_msg;
    home_msg.smallstatus = 1;
    carousel_pub->publish(home_msg);

    // Get the style context of the button
    auto style_context = smallbutton->get_style_context();

    // Toggle between "energized" (green) and "not_energized" (red) states
    if (style_context->has_class("on")) {
        style_context->remove_class("on");
        style_context->add_class("off");  // Change to not energized (red)
    } else {
        style_context->remove_class("off");
        style_context->add_class("on");  // Change to energized (green)
    }
}

void ScienceHMINode::spectroClicked() {
    rover_msgs::msg::ScienceModule home_msg;
    home_msg.spectrostatus = 1;
    spectro_pub->publish(home_msg);

    // Get the style context of the button
    auto style_context = spectrobutton->get_style_context();

    // Toggle between "energized" (green) and "not_energized" (red) states
    if (style_context->has_class("analyzing")) {
        style_context->remove_class("analyzing");
        style_context->add_class("notanalyzing");  // Change to not energized (red)
    } else {
        style_context->remove_class("notanalyzing");
        style_context->add_class("analyzing");  // Change to energized (green)
    }
}

void ScienceHMINode::agPowerClicked() {
    rover_msgs::msg::ScienceModule home_msg;
    home_msg.agpowerstatus = 1;
    agitator_pub->publish(home_msg);

    // Get the style context of the button
    auto style_context = agitatorpowerbutton->get_style_context();

    // Toggle between "energized" (green) and "not_energized" (red) states
    if (style_context->has_class("on")) {
        style_context->remove_class("on");
        style_context->add_class("off");  // Change to not energized (red)
    } else {
        style_context->remove_class("off");
        style_context->add_class("on");  // Change to energized (green)
    }
}

void ScienceHMINode::light1Clicked() {
    rover_msgs::msg::ScienceModule home_msg;
    home_msg.light1status = 1;
    light_pub->publish(home_msg);

    // Get the style context of the button
    auto style_context = light1button->get_style_context();

    // Toggle between "energized" (green) and "not_energized" (red) states
    if (style_context->has_class("on")) {
        style_context->remove_class("on");
        style_context->add_class("off");  // Change to not energized (red)
    } else {
        style_context->remove_class("off");
        style_context->add_class("on");  // Change to energized (green)
    }
}

void ScienceHMINode::ligth2Clicked() {
    rover_msgs::msg::ScienceModule home_msg;
    home_msg.light2status = 1;
    light_pub->publish(home_msg);

    // Get the style context of the button
    auto style_context = light2button->get_style_context();

    // Toggle between "energized" (green) and "not_energized" (red) states
    if (style_context->has_class("on")) {
        style_context->remove_class("on");
        style_context->add_class("off");  // Change to not energized (red)
    } else {
        style_context->remove_class("off");
        style_context->add_class("on");  // Change to energized (green)
    }
}





void ScienceHMINode::handleTextboxInput() {
    std::string input_text = indexnumberentry->get_text();
    try {
        int index = std::stoi(input_text);
        setCarouselIndex(index);
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Invalid input for carousel index.");
    }
}



// Set carousel index (0-15)
void ScienceHMINode::setCarouselIndex(int index) {
    rover_msgs::msg::ScienceModule home_msg;
    
    if (index >= 0 && index <= 15) {
        home_msg.carouselindex = index;
        carousel_pub->publish(home_msg);
        RCLCPP_INFO(this->get_logger(), "Carousel Index Set: %d", index);
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid Carousel Index! Must be between 0-15.");
    }
}
