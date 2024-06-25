#include <RoverHMI.h>
bool MainHMINode::handleSubsystemStatusGridDraw(const Cairo::RefPtr<Cairo::Context>& context){
    this->comms_misc_status_label->setLabel("Meow");

    return true;
}