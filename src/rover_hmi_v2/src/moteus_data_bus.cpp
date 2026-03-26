#include "moteus_data_bus.h"
#include <cmath>

MoteusDataBus::MoteusDataBus(QObject* parent) : QObject(parent) {}

void MoteusDataBus::logCmd(const QString& cmd) {
    emit commandLogged(cmd);
}

void MoteusDataBus::start() {
    transport_ = mot::Controller::MakeSingletonTransport({});

    for (int id = 1; id <= NUM_MOTORS; id++) {
        mot::Controller::Options opts;
        opts.id = id;
        controllers_.push_back(std::make_shared<mot::Controller>(opts));
    }

    elapsed_.start();

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &MoteusDataBus::poll);
    timer_->start(100);

    logCmd("# transport initialized, polling 6 motors at 10 Hz");
}

void MoteusDataBus::stop() {
    if (timer_) {
        timer_->stop();
    }
    logCmd("# polling stopped");
}

void MoteusDataBus::sendPosition(int motor_id, double position,
                                  double velocity, double max_torque) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    pending_commands_.push_back({motor_id, false, position, velocity, max_torque});

    // Log tview-style command
    auto fmt = [](double v) -> QString {
        return std::isnan(v) ? "nan" : QString::number(v, 'f', 3);
    };
    logCmd(QString("%1> d pos %2 %3 %4")
           .arg(motor_id).arg(fmt(position)).arg(fmt(velocity)).arg(fmt(max_torque)));
}

void MoteusDataBus::sendStop(int motor_id) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    pending_commands_.push_back({motor_id, true, 0, 0, 0});
    logCmd(QString("%1> d stop").arg(motor_id));
}

void MoteusDataBus::sendStopAll() {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    for (int id = 1; id <= NUM_MOTORS; id++) {
        pending_commands_.push_back({id, true, 0, 0, 0});
    }
    logCmd("A> d stop");
}

void MoteusDataBus::poll() {
    // Grab any pending commands
    std::vector<PendingCommand> cmds;
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmds.swap(pending_commands_);
    }

    std::vector<mot::CanFdFrame> frames;

    for (int i = 0; i < NUM_MOTORS; i++) {
        int id = i + 1;

        // Check if there's a command for this motor
        const PendingCommand* cmd = nullptr;
        for (const auto& c : cmds) {
            if (c.motor_id == id) cmd = &c;
        }

        if (cmd && cmd->is_stop) {
            frames.push_back(controllers_[i]->MakeStop());
        } else if (cmd) {
            mot::PositionMode::Command pos_cmd;
            pos_cmd.position = cmd->position;       // NAN = hold current
            pos_cmd.velocity = cmd->velocity;        // NAN = no velocity target
            if (!std::isnan(cmd->max_torque))
                pos_cmd.maximum_torque = cmd->max_torque;
            frames.push_back(controllers_[i]->MakePosition(pos_cmd));
        } else {
            frames.push_back(controllers_[i]->MakeQuery());
        }
    }

    std::vector<mot::CanFdFrame> replies;
    transport_->BlockingCycle(frames.data(), frames.size(), &replies);

    double now = elapsed_.elapsed() / 1000.0;
    std::array<MotorState, NUM_MOTORS> states{};

    for (const auto& frame : replies) {
        int id = frame.source;
        if (id < 1 || id > NUM_MOTORS) continue;

        auto r = mot::Query::Parse(frame.data, frame.size);
        auto& s = states[id - 1];
        s.id = id;
        s.mode = static_cast<int>(r.mode);
        s.fault = static_cast<int>(r.fault);
        s.position = r.position;
        s.velocity = r.velocity;
        s.torque = r.torque;
        s.voltage = r.voltage;
        s.temperature = r.temperature;
        s.timestamp = now;
    }

    emit telemetryUpdated(states);
}
