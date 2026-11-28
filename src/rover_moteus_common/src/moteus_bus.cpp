#include <rover_moteus_common/moteus_bus.h>
#include <rover_moteus_common/moteus_protocol.h>


// =============================================================================
// MoteusBus implementation
// =============================================================================

MoteusBus::MoteusBus(const MoteusBusOptions& opts) {
    opts_ = opts;
    numMotors = opts.can_ids.size();
    telem_.resize(numMotors);
    reinit();
}

// ---------------------------------------------------------------------------
// reinit — re-create owned transport + controllers after calibration.
// Uses TransportRegistry (not singleton) so the new transport is fully owned.
// ---------------------------------------------------------------------------
void MoteusBus::reinit() {
    controllers_.clear();
    transport_.reset();  // fd is now closed — refcount hit 0
    
    // transport_ = mot::TransportRegistry::singleton().make({}).first;
    transport_ = mot::TransportRegistry::singleton().make(opts_.transport_args).first; // FORCE TO 1mb 
    id_to_index_.assign(128, -1);
    for (int i = 0; i < numMotors; i++) id_to_index_[opts_.can_ids[i]] = i;
    for (int i = 0; i < numMotors; i++) {
        
        mot::Controller::Options opts;
        opts.id        = opts_.can_ids[i];
        opts.transport = transport_;
        opts.query_format.q_current = mot::kFloat;
        opts.query_format.power     = mot::kFloat;
        if (opts_.query_aux2) { opts.query_format.aux2_gpio = mot::kInt8; }
        opts.position_format.velocity_limit = mot::kFloat;   
        opts.position_format.accel_limit    = mot::kFloat;  
        controllers_.push_back(std::make_shared<mot::Controller>(opts));
    }
}


const std::vector<MotorTelem>& MoteusBus::cycle(const std::vector<MotorCommand>& motor_cmds) {
    // build one frame for each motor
    std::vector<mot::CanFdFrame> frames;
    for (int m = 0; m < numMotors; m++) {
        if (!opts_.enabled.empty() && !opts_.enabled[m]) continue;
        auto& c = motor_cmds[m];
        if      (c.active && c.is_stop) frames.push_back(MoteusProtocol::makeStopFrame(*controllers_[m]));
        else if (c.active)              frames.push_back(MoteusProtocol::makePositionFrame(
                                            *controllers_[m], c.position, c.velocity,
                                            c.max_velocity, c.max_acceleration));
        else                            frames.push_back(MoteusProtocol::makeQueryFrame(*controllers_[m]));
    }

    // sends every frame
    std::vector<mot::CanFdFrame> replies;
    transport_->BlockingCycle(frames.data(), frames.size(), &replies);

    for (auto& t : telem_) t.connected = false;
    for (const auto& frame : replies) {
        int id = frame.source;
        if (id < 0 || id >= (int)id_to_index_.size() || id_to_index_[id] < 0) continue; 
        auto r = MoteusProtocol::parseReply(frame);
        auto& t = telem_[id_to_index_[id]];
        t.position=r.position; t.velocity=r.velocity; t.torque=r.torque; t.voltage=r.voltage;
        t.temperature=r.temperature;
        t.q_current = std::isnan(r.q_current)?0.0f:(float)r.q_current;
        t.power     = std::isnan(r.power)?0.0f:(float)r.power;
        t.mode=(int)r.mode; t.fault=(int)r.fault; t.connected=true;
        if (opts_.query_aux2) { t.limit_switch = (r.aux2_gpio > 0); }
    }

    return telem_;
}

// sends command to a controller
std::string MoteusBus::diagnostic(uint8_t index, const std::string& cmd) {
    return controllers_[index]->DiagnosticCommand(cmd);
}

bool MoteusBus::isConnected(uint8_t index) {
    return controllers_[index]->SetQuery().has_value();
}