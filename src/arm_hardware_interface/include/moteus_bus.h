#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <memory>
#include <cstdint>
#include <cstdio>

#include "moteus_protocol.h"                       // MoteusProtocol::makeFrame(), parseReply()
#include <rover_arm_common/arm_commands.h>
#include <rover_arm_common/arm_telemetry.h>  

struct MoteusBusOptions {
 // TODO later
 // will specify things like CAN IDS
 // what motors are disabled
 // --can-disable-brs
};

class MoteusBus {
    public:
        MoteusBus(int motors);

        // Builds one frame per motor, sends them all then decode their replies into MotorTelem
        const std::vector<MotorTelem>& cycle(const std::vector<MotorCommand>&  motor_cmds); 

        // Send DiagnosticCommand to controller
        std::string diagnostic(uint8_t index, const std::string& cmd);

        bool isConnected(uint8_t index);
        
        // Re-create transport_ and controllers_ (called after moteus_tool exits).
        void reinit(); 
    private:
        
        
        int numMotors;
        std::vector<MotorTelem> telem_;
        std::shared_ptr<mot::Transport>              transport_;
        std::vector<std::shared_ptr<mot::Controller>> controllers_;  // index = motor_id - 1

};