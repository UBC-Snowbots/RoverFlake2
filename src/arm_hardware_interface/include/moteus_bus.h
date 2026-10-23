#pragma once


class MoteusBus {
    public:
        MoteusBus();

        // Builds one frame per motor, sends them all then decode their replies into MotorTelem
        void cycle(); 

        // zero motors position
        void zero(uint8_t index);

        void set_position(uint8_t index, float position_revs);

        // Send DiagnosticCommand to controller
        std::string diagnostic(uint8_t index, const std::string& cmd);
        

    private:
        // Re-create transport_ and controllers_ (called after moteus_tool exits).
        void reinit(); 
        
        
        
        std::shared_ptr<mot::Transport>              transport_;
        std::vector<std::shared_ptr<mot::Controller>> controllers_;  // index = motor_id - 1

};