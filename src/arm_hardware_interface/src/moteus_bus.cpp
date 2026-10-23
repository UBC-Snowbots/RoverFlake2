#include "moteus_bus.h"
#include "moteus_protocol.h"

// =============================================================================
// MoteusBus implementation
// =============================================================================

MoteusBus::MoteusBus() {
    reinit();
}

void MoteusBus::reinit() {
    controllers_.clear();
    transport_.reset();  // fd is now closed — refcount hit 0

    // transport_ = mot::TransportRegistry::singleton().make({}).first;
    transport_ = mot::TransportRegistry::singleton().make({"--can-disable-brs"}).first; // FORCE TO 1mb

    for (int id = 1; id <= NUM_MOTORS; id++) {
        mot::Controller::Options opts;
        opts.id        = id;
        opts.transport = transport_;
        opts.query_format.q_current = mot::kFloat;
        opts.query_format.power     = mot::kFloat;
        opts.query_format.aux2_gpio = mot::kInt8;
        opts.position_format.velocity_limit = mot::kFloat;   
        opts.position_format.accel_limit    = mot::kFloat;  
        controllers_.push_back(std::make_shared<mot::Controller>(opts));
    }
}


void MoteusBus::cycle() {
    // build one frame for each motor
    std::vector<mot::CanFdFrame> frames;
    for (int m = 0; m < NUM_MOTORS; m++) {
        if(m == 3) continue; // A4 DISABLED
        auto& c = motor_cmds_[m];
        if      (c.active && c.is_stop) frames.push_back(MoteusProtocol::makeStopFrame(*controllers_[m]));
        else if (c.active)              frames.push_back(MoteusProtocol::makePositionFrame(
                                            *controllers_[m], c.position, c.velocity,
                                            c.max_velocity, c.max_acceleration));
        else                            frames.push_back(MoteusProtocol::makeQueryFrame(*controllers_[m]));
    }

    // sends every frame
    std::vector<mot::CanFdFrame> replies;
    transport_->BlockingCycle(frames.data(), frames.size(), &replies);

    for (const auto& frame : replies) {
        int id = frame.source;
        if (id < 1 || id > NUM_MOTORS) continue; 
        auto r = MoteusProtocol::parseReply(frame);
        auto& t = telem_[id - 1];
        t.position=r.position; t.velocity=r.velocity; t.torque=r.torque; t.voltage=r.voltage;
        t.temperature=r.temperature;
        t.q_current = std::isnan(r.q_current)?0.0f:(float)r.q_current;
        t.power     = std::isnan(r.power)?0.0f:(float)r.power;
        t.mode=(int)r.mode; t.fault=(int)r.fault; t.connected=true;
        t.limit_switch = (r.aux2_gpio > 0);
        axes[id - 1].limit_switch = t.limit_switch;
    }
}


// Zero position of a motor.
void MoteusBus::zero(uint8_t index)
{
    int i = static_cast<int>(index);
    controllers_[i]->DiagnosticCommand("d exact 0");
}

void MoteusBus::set_position(uint8_t index, float position_revs)
{
  int i = static_cast<int>(index);

  char cmd[32];
  std::snprintf(cmd, sizeof(cmd), "d exact %f", position_revs);
  controllers_[i]->DiagnosticCommand(cmd);
}

// sends command to a controller
std::string MoteusBus::diagnostic(uint8_t index, const std::string& cmd) {
    return controllers_[i]->DiagnosticCommand(cmd);
}

bool MoteusBus::isConnected(mot::Controller& ctrl) {
    return ctrl.SetQuery();
}