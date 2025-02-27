#include <RoverHMI.h>
bool MainHMINode::handleSubsystemStatusGridDraw(const Cairo::RefPtr<Cairo::Context>& context){
    this->subsys_grid.comms_online_status_label->set_text(this->subsys_grid.comms_online_status_string.c_str());
    this->subsys_grid.comms_misc_status_label->set_text(this->subsys_grid.comms_misc_status_string.c_str()); //?passing c++ string seems to work the same as a c string
   
    this->subsys_grid.arm_online_status_label->set_text(this->subsys_grid.arm_online_status_string.c_str()); 
    this->subsys_grid.arm_misc_status_label->set_text(this->subsys_grid.arm_misc_status_string.c_str()); 

    this->subsys_grid.drive_online_status_label->set_text(this->subsys_grid.drive_online_status_string.c_str()); 
    this->subsys_grid.drive_misc_status_label->set_text(this->subsys_grid.drive_misc_status_string.c_str()); 

    this->subsys_grid.ptz_online_status_label->set_text(this->subsys_grid.ptz_online_status_string.c_str()); 
    this->subsys_grid.ptz_misc_status_label->set_text(this->subsys_grid.ptz_misc_status_string.c_str()); 

    this->subsys_grid.lights_online_status_label->set_text(this->subsys_grid.lights_online_status_string.c_str()); 
    this->subsys_grid.lights_misc_status_label->set_text(this->subsys_grid.lights_misc_status_string.c_str()); 

    return true;
}