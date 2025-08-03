#include <science_moduleHMI.h>

//for Rinse for example
//i want to do SV2 then wait like 15 seconds then close and then do 5 sec wait then do SV1, and the buttons should appear and the message should appear after that wait period
//
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
    estopbutton->get_style_context()->add_class("not_active");

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


void ScienceHMINode::updateValveButton(Gtk::Button* button, bool energized) {
    if (energized) {
        button->get_style_context()->remove_class("not_energized");
        button->get_style_context()->add_class("energized");
    } else {
        button->get_style_context()->remove_class("energized");
        button->get_style_context()->add_class("not_energized");
    }
}

void ScienceHMINode::updateAGButton(Gtk::Button* button, bool energized) {
    if (energized) {
        button->get_style_context()->remove_class("off");
        button->get_style_context()->add_class("on");
    } else {
        button->get_style_context()->remove_class("on");
        button->get_style_context()->add_class("off");
    }
}

void ScienceHMINode::updatePumpUI(Gtk::Label* label, Gtk::Button* forwardButton, int pumpStatus) {
    if (pumpStatus == 2) {  // forward
            p1forward->get_style_context()->remove_class("not_active");
            p1forward->get_style_context()->add_class("active");

            p1reverse->get_style_context()->remove_class("not_active");
            p1reverse->get_style_context()->remove_class("active");

            pumpstatuslabel->set_text("Forward");
            pumpstatuslabel->get_style_context()->remove_class("label-reverse");
            pumpstatuslabel->get_style_context()->remove_class("label-stop");
            pumpstatuslabel->get_style_context()->add_class("label-forward");
    } else if(pumpStatus == 1){
        p1forward->get_style_context()->remove_class("active");
        p1forward->get_style_context()->remove_class("not_active");
        p1forward->get_style_context()->add_class("not_active");

        p1reverse->get_style_context()->remove_class("not_active");
        p1reverse->get_style_context()->add_class("active");

        pumpstatuslabel->set_text("Reverse");
        pumpstatuslabel->get_style_context()->remove_class("label-stop");
        pumpstatuslabel->get_style_context()->remove_class("label-forward");
        pumpstatuslabel->get_style_context()->add_class("label-reverse");
    }
    else{
        p1reverse->get_style_context()->remove_class("not_active");
        p1reverse->get_style_context()->remove_class("active");

        p1forward->get_style_context()->remove_class("active");
            p1forward->get_style_context()->remove_class("not_active");
            p1forward->get_style_context()->add_class("not_active");

            p1stop->get_style_context()->remove_class("not_active");
            p1stop->get_style_context()->add_class("active");
            pumpstatuslabel->set_text("Stopped");
            pumpstatuslabel->get_style_context()->remove_class("label-reverse");
            pumpstatuslabel->get_style_context()->remove_class("label-forward");
            pumpstatuslabel->get_style_context()->add_class("label-stop");

    }
}

void ScienceHMINode::rinseSequence() {
    rinse_step = 0;

    // Customize delay between each step in milliseconds
    std::vector<int> delays_ms = {2000, 1000, 4000, 2000};

    // Use shared_ptr to safely allow recursive lambda
    auto advance_step = std::make_shared<std::function<void()>>();
    
    *advance_step = [this, advance_step, delays_ms]() mutable {
        if (home_msg.sequenceselection != 1) {
            RCLCPP_WARN(this->get_logger(), "Rinse sequence aborted due to sequence change.");
            return;
        }

        switch (rinse_step) {
            case 0:
                home_msg.sv2status = 1;
                valve_pub->publish(home_msg);
                updateValveButton(sv2button, true);
                break;

            case 1:
                home_msg.sv2status = 0;
                home_msg.sv1status = 1;
                home_msg.sv4status = 1;
                home_msg.p1status = 2;
                valve_pub->publish(home_msg);
                pump_pub->publish(home_msg);
                updateValveButton(sv2button, false);
                updateValveButton(sv1button, true);
                updateValveButton(sv4button, true);
                updatePumpUI(pumpstatuslabel, p1forward, 2);
                break;

            case 2:
                home_msg.sv1status = 0;
                home_msg.sv4status = 0;
                home_msg.p1status = 0;
                valve_pub->publish(home_msg);
                pump_pub->publish(home_msg);
                updateValveButton(sv1button, false);
                updateValveButton(sv4button, false);
                updatePumpUI(pumpstatuslabel, p1forward, 0);

                if (home_msg.sequenceselection == 1) {
                    rinsebutton->get_style_context()->remove_class("active");
                    rinsebutton->get_style_context()->add_class("not_active");
                }

                resetSystem();
                return;
        }

        int current_delay = (rinse_step < delays_ms.size()) ? delays_ms[rinse_step] : 3000;
        rinse_step++;

        Glib::signal_timeout().connect_once(*advance_step, current_delay);
    };

    // Call the first step
    //Delay the start explicitly
    int initial_delay = delays_ms[0];
    Glib::signal_timeout().connect_once(*advance_step, initial_delay);
    //(*advance_step)();
}


void ScienceHMINode::agitatorSequence() {
    ag_step = 0;

    std::vector<int> delays_ms = {2000, 1000, 5000, 2000};  // Customize delays here

    auto advance_step = std::make_shared<std::function<void()>>();

    *advance_step = [this, advance_step, delays_ms]() mutable {
        if (home_msg.sequenceselection != 2) {
            RCLCPP_WARN(this->get_logger(), "Agitator sequence aborted due to sequence change.");
            return;
        }

        switch (ag_step) {
            case 0:
                home_msg.sv2status = 1;
                valve_pub->publish(home_msg);
                updateValveButton(sv2button, true);
                break;

            case 1:
                home_msg.sv2status = 0;
                home_msg.agpowerstatus = 1;
                valve_pub->publish(home_msg);
                agitator_pub->publish(home_msg);
                updateValveButton(sv2button, false);
                updateAGButton(agitatorpowerbutton, true);
                break;

            case 2:
                home_msg.agpowerstatus = 0;
                agitator_pub->publish(home_msg);
                updateAGButton(agitatorpowerbutton, false);

                if (home_msg.sequenceselection == 2) {
                    agitatorbutton->get_style_context()->remove_class("active");
                    agitatorbutton->get_style_context()->add_class("not_active");
                }

                resetSystem();
                return;
        }

        int current_delay = (ag_step < delays_ms.size()) ? delays_ms[ag_step] : 3000;
        ag_step++;

        Glib::signal_timeout().connect_once(*advance_step, current_delay);
    };

    //Delay the start explicitly
    int initial_delay = delays_ms[0];
    Glib::signal_timeout().connect_once(*advance_step, initial_delay);

    // Call first step
    //(*advance_step)();
}

void ScienceHMINode::processSequence() {

    carousel_index_ = 0;
    home_msg.carouselindex = 0;
    label_carousel->set_text(std::to_string(carousel_index_));
    carousel_pub->publish(home_msg);
    

    process_step = 0;

    std::vector<int> delays_ms = {2000, 4000, 3000};  // Customize delays per step

    auto advance_step = std::make_shared<std::function<void()>>();

    *advance_step = [this, advance_step, delays_ms]() mutable {
        if (home_msg.sequenceselection != 3) {
            RCLCPP_WARN(this->get_logger(), "Process sequence aborted due to sequence change.");
            return;
        }

        switch (process_step) {
            case 0:
                home_msg.sv1status = 1;
                home_msg.p1status = 2;
                valve_pub->publish(home_msg);
                pump_pub->publish(home_msg);
                updateValveButton(sv1button, true);
                updatePumpUI(pumpstatuslabel, p1forward, 2);
                break;

            case 1:
                home_msg.sv1status = 0;
                home_msg.p1status = 0;
                valve_pub->publish(home_msg);
                pump_pub->publish(home_msg);
                updateValveButton(sv1button, false);
                updatePumpUI(pumpstatuslabel, p1forward, 0);

                if (home_msg.sequenceselection == 3) {
                    processbutton->get_style_context()->remove_class("active");
                    processbutton->get_style_context()->add_class("not_active");
                }

                resetSystem();
                return;
        }

        int current_delay = (process_step < delays_ms.size()) ? delays_ms[process_step] : 3000;
        process_step++;

        Glib::signal_timeout().connect_once(*advance_step, current_delay);
    };

    // Delay the start explicitly
    int initial_delay = delays_ms[0];
    Glib::signal_timeout().connect_once(*advance_step, initial_delay);

    //(*advance_step)();  // Start the first step
}

bool is_purging;

void ScienceHMINode::purgeSequence() {
    is_purging = true;
    process_step = 0;

    std::vector<int> delays_ms = {2000, 3000, 6000, 3000};  // Customize delays per step

    auto advance_step = std::make_shared<std::function<void()>>();

    *advance_step = [this, advance_step, delays_ms]() mutable {
        if (home_msg.sequenceselection != 4) {
            RCLCPP_WARN(this->get_logger(), "Purge sequence aborted due to sequence change.");
            return;
        }

        switch (process_step) {

            case 0:
                home_msg.sv3status = 1;
                home_msg.p1status = 1;
                valve_pub->publish(home_msg);
                pump_pub->publish(home_msg);
                updateValveButton(sv3button, true);
                updatePumpUI(pumpstatuslabel, p1reverse, 1);

                break;
            
            case 1:
                home_msg.sv3status = 1;
                home_msg.p1status = 2;
                valve_pub->publish(home_msg);
                pump_pub->publish(home_msg);
                updateValveButton(sv3button, true);
                updatePumpUI(pumpstatuslabel, p1forward, 2);

                break;
                

            case 2:
                home_msg.p1status = 0;
                pump_pub->publish(home_msg);
                updatePumpUI(pumpstatuslabel, p1forward, 0);

                if (home_msg.sequenceselection == 4) {
                    processbutton->get_style_context()->remove_class("active");
                    processbutton->get_style_context()->add_class("not_active");
                }

                resetSystem();
                is_purging = false;
                return;
        }

        int current_delay = (process_step < delays_ms.size()) ? delays_ms[process_step] : 3000;
        process_step++;

        Glib::signal_timeout().connect_once(*advance_step, current_delay);
    };

    //Delay the start explicitly
    int initial_delay = delays_ms[0];
    Glib::signal_timeout().connect_once(*advance_step, initial_delay);

    //(*advance_step)();  // Start the first step
}


void ScienceHMINode::setSequence(bool pressed, int button) {
    //rover_msgs::msg::ScienceModule home_msg;
    estopbutton->get_style_context()->add_class("not_active");

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
            resetSystem();

            RCLCPP_INFO(this->get_logger(), "rinse");
            estopbutton->get_style_context()->remove_class("not_energized");
            estopbutton->get_style_context()->add_class("not_active");
            home_msg.sequenceselection = 1; 
            rinsebutton->get_style_context()->remove_class("not_active");
            rinsebutton->get_style_context()->add_class("active");
            rinseSequence();
            break;
        case sequence_status::agitator: 
            resetSystem();

            RCLCPP_INFO(this->get_logger(), "agitator");
            estopbutton->get_style_context()->remove_class("not_energized");
            estopbutton->get_style_context()->add_class("not_active");
            home_msg.sequenceselection = 2; 
            agitatorbutton->get_style_context()->remove_class("not_active");
            agitatorbutton->get_style_context()->add_class("active");
            agitatorSequence();
            break;
        case sequence_status::process: 
            resetSystem();

            RCLCPP_INFO(this->get_logger(), "process");
            estopbutton->get_style_context()->remove_class("not_energized");
            estopbutton->get_style_context()->add_class("not_active");
            home_msg.sequenceselection = 3; 
            processbutton->get_style_context()->remove_class("not_active");
            processbutton->get_style_context()->add_class("active");
            processSequence();
            break;
        case sequence_status::purge: 
            resetSystem();

            RCLCPP_INFO(this->get_logger(), "process");
            estopbutton->get_style_context()->remove_class("not_energized");
            estopbutton->get_style_context()->add_class("not_active");
            home_msg.sequenceselection = 4; 
            purgebutton->get_style_context()->remove_class("not_active");
            purgebutton->get_style_context()->add_class("active");
            purgeSequence();
            break;
    }

    sequence_pub->publish(home_msg);
}




// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::SV1clicked() {

    home_msg.sv1status = toggleButtonStyle(sv1button, "energized", "not_energized");
    valve_pub->publish(home_msg);


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

void ScienceHMINode::setPump(bool pressed, int button) {
    estopbutton->get_style_context()->add_class("not_active");

    PumpStatus status = static_cast<PumpStatus>(button);

    // Reset all pump buttons' styles
    for (auto b : pump_buttons) {
        b->get_style_context()->remove_class("active");
        b->get_style_context()->remove_class("not_active");
        b->get_style_context()->add_class("not_active");
    }

    // Update based on selected pump status
    switch (status) {
        case PumpStatus::Reverse:
            RCLCPP_INFO(this->get_logger(), "Pump Reverse");
            home_msg.p1status = 1;
            p1forward->get_style_context()->remove_class("active");
            p1forward->get_style_context()->remove_class("not_active");
            p1forward->get_style_context()->add_class("not_active");

            p1reverse->get_style_context()->remove_class("not_active");
            p1reverse->get_style_context()->add_class("active");

            pumpstatuslabel->set_text("Reverse");
            pumpstatuslabel->get_style_context()->remove_class("label-stop");
            pumpstatuslabel->get_style_context()->remove_class("label-forward");
            pumpstatuslabel->get_style_context()->add_class("label-reverse");
            break;
        case PumpStatus::Stop:
            RCLCPP_INFO(this->get_logger(), "Pump Stop");
            home_msg.p1status = 0;
            p1forward->get_style_context()->remove_class("active");
            p1forward->get_style_context()->remove_class("not_active");
            p1forward->get_style_context()->add_class("not_active");

            p1stop->get_style_context()->remove_class("not_active");
            p1stop->get_style_context()->add_class("active");
            pumpstatuslabel->set_text("Stopped");
            pumpstatuslabel->get_style_context()->remove_class("label-reverse");
            pumpstatuslabel->get_style_context()->remove_class("label-forward");
            pumpstatuslabel->get_style_context()->add_class("label-stop");
            break;
        case PumpStatus::Forward:
            RCLCPP_INFO(this->get_logger(), "Pump Forward");
            home_msg.p1status = 2;
            p1forward->get_style_context()->remove_class("not_active");
            p1forward->get_style_context()->add_class("active");
            pumpstatuslabel->set_text("Forward");
            pumpstatuslabel->get_style_context()->remove_class("label-reverse");
            pumpstatuslabel->get_style_context()->remove_class("label-stop");
            pumpstatuslabel->get_style_context()->add_class("label-forward");
            break;
    }

    pump_pub->publish(home_msg);
}


bool osf1_unblocked = false;
bool osf2_unblocked = false;

// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::OSF1clicked() {
    osf1_unblocked = !osf1_unblocked;
    estopbutton->get_style_context()->add_class("not_active");

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
    estopbutton->get_style_context()->add_class("not_active");

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

    updateOFSBlockedLable();
}

bool is_estopped;
void ScienceHMINode::updateOFSBlockedLable(){

    if (is_purging) return;

    if (!osf1_unblocked || !osf2_unblocked) {
        RCLCPP_ERROR(this->get_logger(), "OSF1 or OSF2 is blocked. Triggering emergency stop.");

        ofsblockedlabel->set_text("ERROR: OSF BLOCKED");
        ofsblockedlabel->get_style_context()->remove_class("norm");
        ofsblockedlabel->get_style_context()->add_class("error");

        is_estopped = true;
        resetSystem();
    }

    else{
        ofsblockedlabel->set_text("OFS status");
        ofsblockedlabel->get_style_context()->remove_class("error");
        ofsblockedlabel->get_style_context()->add_class("norm");
    }

}


// Toggle valve states (Only two states: Energized (green) and Not Energized (red))
void ScienceHMINode::prevClicked() {
    // Wrap from 0 → 15
    carousel_index_ = (carousel_index_ == 0) ? 15 : carousel_index_ - 1;

    estopbutton->get_style_context()->add_class("not_active");
   

    updateCarouselIndexLabel();

    home_msg.carouseldir = -1;
    carousel_pub->publish(home_msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    

    home_msg.carouseldir= 0;
    carousel_pub->publish(home_msg);
}


void ScienceHMINode::nextClicked() {
    // Wrap from 15 → 0
    carousel_index_ = (carousel_index_ + 1) % 16;

    //update button:
    estopbutton->get_style_context()->add_class("not_active");
    

    // Update the label
    updateCarouselIndexLabel();

    // Set motor direction forward
    home_msg.carouseldir= 1;
    carousel_pub->publish(home_msg);


    // After short delay, stop motor
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // Disable buttons here


    home_msg.carouseldir = 0;
    carousel_pub->publish(home_msg);
}

void ScienceHMINode::updateCarouselIndexLabel() {
    home_msg.carouselindex = carousel_index_;
    label_carousel->set_text(std::to_string(carousel_index_));
    carousel_pub->publish(home_msg);
}

void ScienceHMINode::resetClicked(){
    carousel_index_ = 0;
    home_msg.carouselindex = 0;
    label_carousel->set_text(std::to_string(carousel_index_));
    carousel_pub->publish(home_msg);
}



void ScienceHMINode::largeClicked() {

    estopbutton->get_style_context()->add_class("not_active");
    is_estopped = false;
    process_step = 0;

    std::vector<int> delays_ms = {2000, 1500, 2000};  // Customize delays per step

    auto advance_step = std::make_shared<std::function<void()>>();

    *advance_step = [this, advance_step, delays_ms]() mutable {
        if (is_estopped) {
            RCLCPP_WARN(this->get_logger(), "Advance step aborted due to E-Stop");
            return;
        }

        switch (process_step) {

            case 0:
            estopbutton->get_style_context()->remove_class("not_energized");
            estopbutton->get_style_context()->add_class("not_active");
                largebutton->get_style_context()->remove_class("not_energized");
                largebutton->get_style_context()->add_class("energized");
                home_msg.sv4status = 0;
                home_msg.largestatus = 1;
                break;

            case 1:
                home_msg.sv4status = 1;
                valve_pub->publish(home_msg);
                
                updateValveButton(sv4button, true);
                break;
        

            case 2:
                home_msg.sv4status = 0;
                home_msg.largestatus = 0;
                valve_pub->publish(home_msg);
                updateValveButton(sv4button, false);
                largebutton->get_style_context()->remove_class("energized");
                largebutton->get_style_context()->add_class("not_energized");

                // if (home_msg.sequenceselection == 4) {
                //     processbutton->get_style_context()->remove_class("active");
                //     processbutton->get_style_context()->add_class("not_active");
                // }

                resetSystem();
                return;
        }

        int current_delay = (process_step < delays_ms.size()) ? delays_ms[process_step] : 3000;
        process_step++;

        Glib::signal_timeout().connect_once(*advance_step, current_delay);
    };

    (*advance_step)();  // Start the first step
}


void ScienceHMINode::smallClicked() {

    estopbutton->get_style_context()->add_class("not_active");
    is_estopped = false;
    process_step = 0;

    std::vector<int> delays_ms = {2000, 500, 2000};  // Customize delays per step

    auto advance_step = std::make_shared<std::function<void()>>();

    *advance_step = [this, advance_step, delays_ms]() mutable {
        if (is_estopped) {
            RCLCPP_WARN(this->get_logger(), "Advance step aborted due to E-Stop");
            return;
        }

        switch (process_step) {
            case 0:
                estopbutton->get_style_context()->remove_class("not_energized");
                estopbutton->get_style_context()->add_class("not_active");
                smallbutton->get_style_context()->remove_class("not_energized");
                smallbutton->get_style_context()->add_class("energized");
                updateValveButton(sv4button, false);

                home_msg.sv4status = 0;
                home_msg.smallstatus = 1;
                break;

            case 1:
                home_msg.sv4status = 1;
                valve_pub->publish(home_msg);
                
                updateValveButton(sv4button, true);
                break;
        

            case 2:
                home_msg.sv4status = 0;
                home_msg.smallstatus = 0;
                valve_pub->publish(home_msg);
                updateValveButton(sv4button, false);
                smallbutton->get_style_context()->remove_class("energized");
                smallbutton->get_style_context()->add_class("not_energized");

                // if (home_msg.sequenceselection == 4) {
                //     processbutton->get_style_context()->remove_class("active");
                //     processbutton->get_style_context()->add_class("not_active");
                // }

                resetSystem();
                return;
        }

        int current_delay = (process_step < delays_ms.size()) ? delays_ms[process_step] : 3000;
        process_step++;

        Glib::signal_timeout().connect_once(*advance_step, current_delay);
    };

    (*advance_step)();  // Start the first step
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


void ScienceHMINode::resetSystem() {
    RCLCPP_INFO(this->get_logger(), "Resetting system to default state");

    // Reset sequence buttons
    for (auto b : sequence_buttons) {
        auto ctx = b->get_style_context();
        ctx->remove_class("active");
        ctx->remove_class("not_active");
        ctx->add_class("not_active");
    }
    home_msg.sequenceselection = 0;

    // Reset valves (SV1-SV4)
    std::vector<Gtk::Button*> valves = {sv1button, sv2button, sv3button, sv4button};
    for (auto& valve : valves) {
        auto ctx = valve->get_style_context();
        ctx->remove_class("energized");
        ctx->remove_class("not_energized");
        ctx->add_class("not_energized");
    }
    home_msg.sv1status = 0;
    home_msg.sv2status = 0;
    home_msg.sv3status = 0;
    home_msg.sv4status = 0;

    // Reset pump buttons
    for (auto b : pump_buttons) {
        auto ctx = b->get_style_context();
        ctx->remove_class("active");
        ctx->remove_class("not_active");
        ctx->add_class("not_active");
    }
    p1forward->get_style_context()->add_class("not_active");
    p1reverse->get_style_context()->add_class("not_active");
    p1stop->get_style_context()->add_class("active");
    pumpstatuslabel->set_text("Stopped");
    pumpstatuslabel->get_style_context()->remove_class("label-reverse");
    pumpstatuslabel->get_style_context()->remove_class("label-forward");
    pumpstatuslabel->get_style_context()->add_class("label-stop");
    home_msg.p1status = 0;


    //agitator sequence

    agitatorpowerbutton->get_style_context()->remove_class("on");
    agitatorpowerbutton->get_style_context()->remove_class("on");
    agitatorpowerbutton->get_style_context()->add_class("off");
    home_msg.agpowerstatus = 0;

    // Reset OSF buttons
    // osf1_unblocked = false;
    // osf2_unblocked = false;
    // osf1button->get_style_context()->remove_class("energized");
    // osf1button->get_style_context()->add_class("not_energized");
    // osf2button->get_style_context()->remove_class("energized");
    // osf2button->get_style_context()->add_class("not_energized");
    // home_msg.osf1status = 0;
    // home_msg.osf2status = 0;

    smallbutton->get_style_context()->remove_class("energized");
    largebutton->get_style_context()->remove_class("energized");
    largebutton->get_style_context()->add_class("not_energized");
    smallbutton->get_style_context()->add_class("not_energized");

    home_msg.smallstatus = 0;
    home_msg.largestatus = 0;


    estopbutton->get_style_context()->remove_class("not_active");
    estopbutton->get_style_context()->add_class("not_energized");


    // Publish all resets
    sequence_pub->publish(home_msg);
    valve_pub->publish(home_msg);
    pump_pub->publish(home_msg);

    //osf_pub->publish(home_msg);
}

void ScienceHMINode::stopClicked() {
    RCLCPP_INFO(this->get_logger(), "STOP clicked");
    is_estopped = true;
    resetSystem();
}


bool camera1_on = false;


void ScienceHMINode::cameraFeedChosen(bool clicked, int id)
{
    rover_msgs::msg::CameraVideo msg;
    msg.camera_id = id;

    if (id == 1 && camera1button) {
        camera1_on = !camera1_on;  // toggle the state

        if (camera1_on) {
            camera1button->get_style_context()->remove_class("off");
            camera1button->get_style_context()->add_class("on");
        } else {
            camera1button->get_style_context()->remove_class("on");
            camera1button->get_style_context()->add_class("off");
        }

        camera_video_pub->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Camera 1 toggled %s", camera1_on ? "ON" : "OFF");
    }
}
