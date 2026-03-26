#pragma once

#include <QObject>
#include <QTimer>
#include <QElapsedTimer>
#include <QString>

#include <array>
#include <memory>
#include <vector>
#include <mutex>

#include "moteus.h"

namespace mot = mjbots::moteus;

constexpr int NUM_MOTORS = 6;

struct MotorState {
    int id = 0;
    int mode = 0;
    int fault = 0;
    double position = 0.0;
    double velocity = 0.0;
    double torque = 0.0;
    double voltage = 0.0;
    double temperature = 0.0;
    double timestamp = 0.0;  // seconds since start
};

// Pending command to inject into the next poll cycle
struct PendingCommand {
    int motor_id;
    bool is_stop;
    double position;
    double velocity;
    double max_torque;
};

class MoteusDataBus : public QObject {
    Q_OBJECT
public:
    explicit MoteusDataBus(QObject* parent = nullptr);

    void start();
    void stop();

    // Queue a position command for the next cycle
    // Use NAN for any field to leave it unspecified (tview: nan)
    void sendPosition(int motor_id, double position, double velocity = 0.0,
                      double max_torque = NAN);
    // Queue a stop for the next cycle
    void sendStop(int motor_id);
    // Queue stop for all motors
    void sendStopAll();

signals:
    void telemetryUpdated(const std::array<MotorState, NUM_MOTORS>& states);
    void commandLogged(const QString& cmd);

private:
    void poll();
    void logCmd(const QString& cmd);

    std::shared_ptr<mot::Transport> transport_;
    std::vector<std::shared_ptr<mot::Controller>> controllers_;
    QTimer* timer_ = nullptr;
    QElapsedTimer elapsed_;

    std::mutex cmd_mutex_;
    std::vector<PendingCommand> pending_commands_;
};
