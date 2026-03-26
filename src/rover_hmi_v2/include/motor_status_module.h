#pragma once

#include "gui_module.h"
#include "moteus_data_bus.h"
#include <QLabel>
#include <array>

class MotorStatusModule : public GuiModule {
public:
    std::string name() const override { return "Motor Status"; }
    QWidget* createWidget(QWidget* parent) override;
    void start() override {}
    void stop() override {}

    void setDataBus(MoteusDataBus* bus) override;

private:
    void onTelemetry(const std::array<MotorState, NUM_MOTORS>& states);

    static constexpr int NUM_FIELDS = 7;
    QLabel* cells_[NUM_MOTORS][NUM_FIELDS] = {};
    QLabel* status_ = nullptr;
};
