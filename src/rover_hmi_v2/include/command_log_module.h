#pragma once

#include "gui_module.h"
#include "moteus_data_bus.h"

#include <QTextEdit>

class CommandLogModule : public GuiModule {
public:
    std::string name() const override { return "Command Log"; }
    QWidget* createWidget(QWidget* parent) override;
    void start() override {}
    void stop() override {}

    void setDataBus(MoteusDataBus* bus) override;

private:
    QTextEdit* log_ = nullptr;
};
