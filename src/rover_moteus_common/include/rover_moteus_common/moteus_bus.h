#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <memory>
#include <cstdint>
#include <cstdio>

#include "moteus_protocol.h"                       // MoteusProtocol::makeFrame(), parseReply()
#include <rover_moteus_common/motor_command.h>
#include <rover_moteus_common/motor_telem.h>  

struct MoteusBusOptions {
    std::vector<int> can_ids; // each CAN ID of motor index i
    std::vector<bool> enabled; // is motor is disabled never sends a frame
    bool query_aux2 = false;
    std::vector<std::string> transport_args = {"--can-disable-brs"};
};

class MoteusBus {
    public:
        MoteusBus(const MoteusBusOptions& opts);


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
        std::vector<std::shared_ptr<mot::Controller>> controllers_;  // index = motor index (CAN ID is opts_.can_ids[i])
        std::vector<int> id_to_index_;
        MoteusBusOptions opts_;

};