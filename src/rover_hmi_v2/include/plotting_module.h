#pragma once

#include "gui_module.h"
#include "moteus_data_bus.h"
#include "plot_widget.h"

#include <QCheckBox>
#include <QPushButton>
#include <QComboBox>

class PlottingModule : public GuiModule {
public:
    std::string name() const override { return "Plots"; }
    QWidget* createWidget(QWidget* parent) override;
    void start() override {}
    void stop() override {}

    void setDataBus(MoteusDataBus* bus) override;

private:
    void onTelemetry(const std::array<MotorState, NUM_MOTORS>& states);

    PlotWidget* pos_plot_ = nullptr;
    PlotWidget* vel_plot_ = nullptr;
    QCheckBox* checks_[NUM_MOTORS] = {};
    QPushButton* pause_btn_ = nullptr;
    bool paused_ = false;
};
