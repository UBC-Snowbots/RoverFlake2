#pragma once

#include "gui_module.h"
#include "moteus_data_bus.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QCheckBox>
#include <array>

constexpr int NUM_ZERO_AXES = 6;

class JogButton : public QPushButton {
    Q_OBJECT
public:
    JogButton(const QString& text, QWidget* parent = nullptr);
signals:
    void jogPressed();
    void jogReleased();
protected:
    void mousePressEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
};

class SendCommandModule : public GuiModule {
public:
    std::string name() const override { return "Send Command"; }
    QWidget* createWidget(QWidget* parent) override;
    void start() override {}
    void stop() override {}

    void setDataBus(MoteusDataBus* bus) override;

private:
    MoteusDataBus* bus_ = nullptr;
    QComboBox* motor_select_ = nullptr;
    QComboBox* cmd_type_ = nullptr;
    QDoubleSpinBox* position_spin_ = nullptr;
    QDoubleSpinBox* velocity_spin_ = nullptr;
    QDoubleSpinBox* torque_spin_ = nullptr;
    QDoubleSpinBox* jog_speed_spin_ = nullptr;
    QCheckBox* pos_enable_ = nullptr;
    QCheckBox* vel_enable_ = nullptr;
    QCheckBox* torque_enable_ = nullptr;

    // Per-axis zero checkboxes (index = motor_id - 1)
    std::array<QCheckBox*, NUM_ZERO_AXES> zero_checks_{};
};
