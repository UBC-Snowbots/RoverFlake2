#pragma once

#include "gui_module.h"
#include "moteus_data_bus.h"

#include <QLabel>
#include <array>

class MotorConfigModule : public GuiModule {
public:
    std::string name() const override { return "Motor Config"; }
    QWidget* createWidget(QWidget* parent) override;
    void start() override {}
    void stop() override {}

    void setDataBus(MoteusDataBus* bus) override;

private:
    void onConfig(const std::array<MotorConfigInfo, NUM_MOTORS>& configs);

    // 6 motors x 8 config fields
    static constexpr int NUM_FIELDS = 8;
    QLabel* cells_[NUM_MOTORS][NUM_FIELDS] = {};
    QLabel* status_ = nullptr;
};
