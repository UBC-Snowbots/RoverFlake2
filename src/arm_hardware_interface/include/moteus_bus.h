#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <memory>
#include <cstdint>
#include <cstdio>

#include "moteus_protocol.h"    
#include <rover_arm_common/arm_commands.h>
#include <rover_arm_common/arm_telemetry.h>  

struct MoteusBusOptions {
 // TO DO later
};

class MoteusBus {
    public:
        MoteusBus(int motors);

        // Builds one frame per motor, sends them all then decode their replies into MotorTelem
        const std::vector<MotorTelem>& cycle(const std::vector<MotorCommand>&  motor_cmds); 

        // zero motors position
        void zero(uint8_t index);

        void set_position(uint8_t index, float position_revs);

        // Send DiagnosticCommand to controller
        std::string diagnostic(uint8_t index, const std::string& cmd);

        bool isConnected(uint8_t index);
        

    private:
        // Re-create transport_ and controllers_ (called after moteus_tool exits).
        void reinit(); 
        
        
        int numMotors;
        std::vector<MotorTelem> telem_;
        std::shared_ptr<mot::Transport>              transport_;
        std::vector<std::shared_ptr<mot::Controller>> controllers_;  // index = motor_id - 1

};