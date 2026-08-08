// task_timers_module.h — "Task Timers"
//
// Named countdown timers (RoverCooked orders) plus Snack Run cadence helpers:
// a 5 s credit-tap cycle bar and a double-tap interval meter.
// Section: Tasks. Pure UI, no ROS. defaultVisible: false.

#pragma once

#include <rover_hmi_core/gui_module.h>

#include <QElapsedTimer>
#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <QVector>

class TaskTimersModule : public rover_hmi_core::GuiModule {
public:
    std::string name()        const override { return "Task Timers"; }
    std::string sectionName() const override { return "Tasks"; }
    bool        defaultVisible() const override { return false; }

    QWidget* createWidget(QWidget* parent) override;

private:
    struct Row {
        QWidget*     widget   = nullptr;
        QLabel*      time_lbl = nullptr;
        QPushButton* toggle   = nullptr;
        qint64 total_ms     = 0;
        qint64 remaining_ms = 0;   // valid while paused
        qint64 end_ms       = 0;   // valid while running (clock_ time)
        bool   running      = false;
    };

    void addTimer(const QString& name, int secs);
    void tick();
    void updateRow(Row* row);

    QVector<Row*> rows_;
    QVBoxLayout*  rows_layout_ = nullptr;
    QLineEdit*    name_in_     = nullptr;
    QSpinBox*     min_in_      = nullptr;
    QSpinBox*     sec_in_      = nullptr;

    // credit-tap cadence
    QPushButton*  cad_btn_     = nullptr;
    QProgressBar* cad_bar_     = nullptr;
    bool          cadence_on_  = false;
    qint64        cadence_start_ = 0;

    // double-tap interval meter
    QLabel*       tap_lbl_     = nullptr;
    qint64        last_tap_ms_ = -1;

    QElapsedTimer clock_;
    QTimer*       tick_timer_  = nullptr;
};
