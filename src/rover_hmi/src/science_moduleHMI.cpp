#include <science_moduleHMI.h>

//ADD BUTTON FOR MJK THING ITS LIKE TO ADD CONCETRATION AND STUFF

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


int ScienceHMINode::toggleButtonStyle(Gtk::Button* btn, const std::string& active_class, const std::string& inactive_class) {
    auto style = btn->get_style_context();
    if (style->has_class(active_class)) {
        style->remove_class(active_class);
        style->add_class(inactive_class);
        return 0;
    } else {
        style->remove_class(inactive_class);
        style->add_class(active_class);
        return 1;
    }
}


void ScienceHMINode::setSequence(bool pressed, int button) {
    //rover_msgs::msg::ScienceModule home_msg;
    sequence_status button_triggered = static_cast<sequence_status>(button);

    // Reset all buttons first
    for (auto b : sequence_buttons) {
        b->get_style_context()->remove_class("active");
        b->get_style_context()->remove_class("not_active");
        b->get_style_context()->add_class("not_active");
    }

    // Then activate the clicked one
    switch(button_triggered){
        case sequence_status::rinse: 
            RCLCPP_INFO(this->get_logger(), "rinse");
            home_msg.sequenceselection = 1; 
            rinsebutton->get_style_context()->remove_class("not_active");
            rinsebutton->get_style_context()->add_class("active");
            break;
        case sequence_status::agitator: 
            RCLCPP_INFO(this->get_logger(), "agitator");
            home_msg.sequenceselection = 2; 
            agitatorbutton->get_style_context()->remove_class("not_active");
            agitatorbutton->get_style_context()->add_class("active");
            break;
        case sequence_status::process: 
            RCLCPP_INFO(this->get_logger(), "process");
            home_msg.sequenceselection = 3; 
            processbutton->get_style_context()->remove_class("not_active");
            processbutton->get_style_context()->add_class("active");
            break;
        case sequence_status::purge: 
            RCLCPP_INFO(this->get_logger(), "purge");
            home_msg.sequenceselection = 4; 
            purgebutton->get_style_context()->remove_class("not_active");
            purgebutton->get_style_context()->add_class("active");
            break;
    }

    sequence_pub->publish(home_msg);
}




// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::SV1clicked() {
    //rover_msgs::msg::ScienceModule home_msg;
    // home_msg.sv1status = 1;
   

    // // Get the style context of the button
    // auto style_context = sv1button->get_style_context();


    home_msg.sv1status = toggleButtonStyle(sv1button, "energized", "not_energized");
    valve_pub->publish(home_msg);


    //style_context->add_class("energized");

    // Toggle between "energized" (green) and "not_energized" (red) states
    // if (style_context->has_class("energized")) {
    //     style_context->remove_class("energized");
    //     style_context->add_class("not_energized");  // Change to not energized (red)
    // } else {
    //     style_context->remove_class("not_energized");
    //     style_context->add_class("energized");  // Change to energized (green)
    // }

    // valve_pub->publish(home_msg);

//     auto ctx = sv1button->get_style_context();
// auto classes = ctx->list_classes();

// std::cout << "🔍 SV1 styles: ";
// for (const auto& cls : classes)
//     std::cout << cls << " ";
// std::cout << std::endl;

}


// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::SV2clicked() {
    //rover_msgs::msg::ScienceModule home_msg;
    home_msg.sv2status = toggleButtonStyle(sv2button, "energized", "not_energized");
    valve_pub->publish(home_msg);

}


// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::SV3clicked() {
    //rover_msgs::msg::ScienceModule home_msg;
    home_msg.sv3status = toggleButtonStyle(sv3button, "energized", "not_energized");
    valve_pub->publish(home_msg);

}


// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::SV4clicked() {
    //rover_msgs::msg::ScienceModule home_msg;
    home_msg.sv4status = toggleButtonStyle(sv4button, "energized", "not_energized");
    valve_pub->publish(home_msg);

}


// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::SVF1clicked() {
    //rover_msgs::msg::ScienceModule home_msg;
    home_msg.svf1status = toggleButtonStyle(svf1button, "energized", "not_energized");
    valve_pub->publish(home_msg);

}


// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::SVF2clicked() {
    //rover_msgs::msg::ScienceModule home_msg;
    home_msg.svf2status = toggleButtonStyle(svf2button, "energized", "not_energized");
    valve_pub->publish(home_msg);

}

// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::P1clicked() {
    //rover_msgs::msg::ScienceModule home_msg;
    

    auto style_context = p1button->get_style_context();
    

    if (style_context->has_class("forward")) {
        style_context->remove_class("forward");
        style_context->add_class("reverse");
        p1button->set_label("P1 (Reverse)");  // Change label to show it's in backward state
        home_msg.p1status = 0;
    } else {
        style_context->remove_class("reverse");
        style_context->add_class("forward");
        p1button->set_label("P1 (Forward)");  // Or just "P1" or any label for forward state
        home_msg.p1status = 1;
    
    }

    osf_pub->publish(home_msg);

}

bool osf1_unblocked = false;
bool osf2_unblocked = false;

// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::OSF1clicked() {
    osf1_unblocked = !osf1_unblocked;
    //rover_msgs::msg::ScienceModule home_msg;

    // Get the style context of the button
    auto style_context = osf1button->get_style_context();

    // Toggle between "energized" (green) and "not_energized" (red) states
    if (!osf1_unblocked) {
        style_context->remove_class("unblocked");
        style_context->add_class("blocked");  // Change to not energized (red)
        home_msg.osf1status = 0;

    } else {
        style_context->remove_class("blocked");
        style_context->add_class("unblocked");  // Change to energized (green)
        home_msg.osf1status = 1;
    }

    osf_pub->publish(home_msg);


    updateStatusLabel();
}

// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::OSF2clicked() {
    osf2_unblocked = !osf2_unblocked;
    //rover_msgs::msg::ScienceModule home_msg;
    
    // Get the style context of the button
    auto style_context = osf2button->get_style_context();

    // Toggle between "energized" (green) and "not_energized" (red) states
     if (!osf2_unblocked) {
        style_context->remove_class("unblocked");
        style_context->add_class("blocked");  // Change to not energized (red)
        home_msg.osf2status = 0;

    } else {
        style_context->remove_class("blocked");
        style_context->add_class("unblocked");  // Change to energized (green)
        home_msg.osf2status = 1;

    }


    pump_pub->publish(home_msg);


    updateStatusLabel();
}

void ScienceHMINode::updateStatusLabel() {
    auto style_context = statuslabel->get_style_context();
    style_context->remove_class("label-blocked");
    style_context->remove_class("label-unblocked");

    if (osf1_unblocked && osf2_unblocked) {
        statuslabel->set_text("UNBLOCKED");
        style_context->add_class("label-unblocked");
    } else {
        statuslabel->set_text("BLOCKED");
        style_context->add_class("label-blocked");
    }
}


// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::prevClicked() {
    //rover_msgs::msg::ScienceModule home_msg;


    home_msg.prevstatus = toggleButtonStyle(prevIndexbutton, "on", "off");
    carousel_pub->publish(home_msg);
}

// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::nextClicked() {
    //rover_msgs::msg::ScienceModule home_msg;

    home_msg.nextstatus = toggleButtonStyle(nextidxbutton, "on", "off");
    carousel_pub->publish(home_msg);
}

// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::largeClicked() {
    //rover_msgs::msg::ScienceModule home_msg;

    home_msg.largestatus = toggleButtonStyle(largebutton, "on", "off");
    carousel_pub->publish(home_msg);

}

// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::smallClicked() {
    //rover_msgs::msg::ScienceModule home_msg;

    home_msg.smallstatus = toggleButtonStyle(smallbutton, "on", "off");
    carousel_pub->publish(home_msg);
}

void ScienceHMINode::spectroClicked() {
    //rover_msgs::msg::ScienceModule home_msg;

    home_msg.spectrostatus = toggleButtonStyle(spectrobutton, "analyzing", "notanalyzing");
    spectro_pub->publish(home_msg);

}

void ScienceHMINode::agPowerClicked() {
    //rover_msgs::msg::ScienceModule home_msg;

    home_msg.agpowerstatus = toggleButtonStyle(agitatorpowerbutton, "on", "off");
    agitator_pub->publish(home_msg);
}

void ScienceHMINode::light1Clicked() {
    //rover_msgs::msg::ScienceModule home_msg;

    home_msg.light1status = toggleButtonStyle(light1button, "on", "off");
    light_pub->publish(home_msg);


}

void ScienceHMINode::ligth2Clicked() {
    //rover_msgs::msg::ScienceModule home_msg;

    home_msg.light2status = toggleButtonStyle(light2button, "on", "off");
    light_pub->publish(home_msg);
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
    //rover_msgs::msg::ScienceModule home_msg;
    
    if (index >= 0 && index <= 15) {
        home_msg.carouselindex = index;
        carousel_pub->publish(home_msg);
        RCLCPP_INFO(this->get_logger(), "Carousel Index Set: %d", index);
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid Carousel Index! Must be between 0-15.");
    }
}




void ScienceHMINode::cameraFeedChosen(bool clicked, int id)
{
    rover_msgs::msg::CameraVideo msg;
    msg.camera_id = id;  // e.g. 1 for camera1button, 2 for camera2button
    camera_video_pub->publish(msg);

    if(id == 1){
        camera1button->get_style_context()->remove_class("not_active");
        camera1button->get_style_context()->add_class("active");
        camera2button->get_style_context()->remove_class("active");
        camera2button->get_style_context()->add_class("not_active");
    }

    else if(id ==2){
        camera2button->get_style_context()->remove_class("not_active");
        camera2button->get_style_context()->add_class("active");
        camera1button->get_style_context()->remove_class("active");
        camera1button->get_style_context()->add_class("not_active");
    }

    RCLCPP_INFO(this->get_logger(), "Camera feed %d chosen", id);
}





//home_msg.data = ....
//do 1, 2

//