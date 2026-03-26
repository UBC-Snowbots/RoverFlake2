#pragma once

#include "gui_module.h"
#include <QWidget>
#include <QProcess>
#include <QTimer>
#include <QEvent>
#include <QVBoxLayout>
#include <QLabel>
#include <string>

class RvizModule : public QObject, public GuiModule {
    Q_OBJECT
public:
    std::string name() const override { return "RViz Simulation"; }
    QWidget* createWidget(QWidget* parent) override;
    void start() override {}
    void stop() override;
    bool defaultVisible() const override { return false; }
    std::function<void(bool)> toggleCallback() override;

    void onToggled(bool visible);

private:
    void launchProcesses();
    void tryEmbed();
    void embedWindow();
    void unembed();

    QWidget* container_ = nullptr;
    QVBoxLayout* layout_ = nullptr;
    QLabel* status_label_ = nullptr;
    QWidget* embedded_widget_ = nullptr;

    QProcess* rsp_process_ = nullptr;
    QProcess* rviz_process_ = nullptr;
    QTimer* embed_timer_ = nullptr;
    int embed_attempts_ = 0;

    std::string robot_description_;
    std::string params_file_;
    unsigned long rviz_wid_ = 0;
    bool embedded_ = false;
    bool launched_ = false;
};
