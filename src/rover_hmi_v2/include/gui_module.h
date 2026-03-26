#pragma once

#include <QWidget>
#include <string>

class MoteusDataBus;

// Base class for GUI modules.
// Subclass this, implement the methods, and add to the modules vector in main.
struct GuiModule {
    virtual ~GuiModule() = default;

    // Display name shown in the module's dock widget
    virtual std::string name() const = 0;

    // Create and return the module's widget. Called once at startup.
    virtual QWidget* createWidget(QWidget* parent) = 0;

    // Called after the window is shown. Start timers/threads here.
    virtual void start() = 0;

    // Called on shutdown.
    virtual void stop() = 0;

    // Override to receive the shared data bus. Called before start().
    virtual void setDataBus(MoteusDataBus*) {}
};
