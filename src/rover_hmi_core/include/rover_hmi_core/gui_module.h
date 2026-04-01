// =============================================================================
// gui_module.h — shared plugin interface for all HMI modules
//
// This header lives in rover_hmi_core so that any package in the workspace can
// depend on it.  pluginlib uses GuiModule as the base class for runtime module
// discovery: the host (hmi_host.cpp, also in rover_hmi_core) calls
// ClassLoader<GuiModule> and receives concrete subclasses from any package that
// has registered them via a plugins.xml descriptor.
//
// To add a new module:
//   1. Subclass GuiModule and implement the pure virtual methods below.
//   2. Register the class in your package's plugins.xml.
//   3. Add pluginlib_export_plugin_description_file() to your CMakeLists.txt.
//   No changes to rover_hmi_core are required — the host discovers it
//   automatically at startup.
// =============================================================================

#pragma once

#include <QWidget>
#include <string>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace rover_hmi_core {

// Base class for all HMI modules.
// Subclass this, implement the pure virtual methods, register via pluginlib.
// The HMI host (rover_hmi executable) discovers and loads modules at runtime — no changes to this package needed.
class GuiModule {
public:
    virtual ~GuiModule() = default;

    // Display name shown in the module's tile and sidebar
    virtual std::string name() const = 0;

    // Layout region hint for initial placement:
    //   "main"   = top-left (large)
    //   "right"  = top-right (stacked vertically)
    //   "bottom" = bottom strip
    virtual std::string layoutHint() const { return "right"; }

    // Create and return the module's widget. Called once at startup.
    virtual QWidget* createWidget(QWidget* parent) = 0;

    // Called after the window is shown. Start timers/subscriptions here if needed.
    virtual void start() {}

    // Called on shutdown. Clean up resources.
    virtual void stop() {}

    // Receives the shared ROS2 node. Create subscriptions/publishers here.
    // Called before createWidget().
    virtual void setNode(rclcpp::Node::SharedPtr /*node*/) {}

    // Override to start hidden (toggled off in sidebar by default).
    virtual bool defaultVisible() const { return true; }

    // Override to receive sidebar toggle events.
    virtual std::function<void(bool)> toggleCallback() { return nullptr; }

    // Override to expose module-specific keybindings shown in the Alt+/ overlay.
    // Each pair is { keys, description }, e.g. { "Alt+R", "Reset motors" }.
    virtual std::vector<std::pair<std::string,std::string>> keybindings() const { return {}; }

    // Section this module belongs to in the sidebar.
    // Modules sharing a section name are grouped together.
    // Alt+1..9 applies only to modules in the currently active section.
    virtual std::string sectionName() const { return "General"; }
};

}  // namespace rover_hmi_core
