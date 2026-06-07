// =============================================================================
// hmi_host.cpp — generic HMI host process
//
// Owns the top-level Qt window, the TilingContainer layout manager, and the
// shared ROS2 node.  Contains no subsystem-specific logic: all domain
// knowledge lives in GuiModule subclasses discovered at runtime via pluginlib.
//
// Module discovery:
//   pluginlib::ClassLoader scans every package in the ament index that has
//   registered a plugins.xml exporting rover_hmi_core::GuiModule.  Any such
//   package will have its modules instantiated and tiled here automatically —
//   no source changes to this file are ever needed when adding new modules.
//
// ROS2 / Qt integration:
//   Qt runs its own event loop (app.exec()) and has no knowledge of ROS2
//   executors.  A 20 ms QTimer calls rclcpp::spin_some() on each tick, which
//   drains the node's callback queue without blocking Qt.  20 ms gives ~50 Hz
//   update rate, which is sufficient for all current sensor visualisations.
// =============================================================================

#include <rover_hmi_core/tiling_container.h>
#include <rover_hmi_core/gui_module.h>
#include <rover_hmi_core/catppuccin.h>

#include <pluginlib/class_loader.hpp>

#include <QApplication>
#include <QMainWindow>
#include <QTimer>

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rover_hmi");

    QApplication app(argc, argv);
    theme::applyGlobalStylesheet(app);

    // Pump ROS2 callbacks from within Qt's event loop.  spin_some() is
    // non-blocking: it processes whatever callbacks are ready and returns
    // immediately, so Qt remains responsive between ticks.
    QTimer spin_timer;
    QObject::connect(&spin_timer, &QTimer::timeout, [&node]() {
        rclcpp::spin_some(node);
    });
    spin_timer.start(20);  // 20 ms ≈ 50 Hz

    QMainWindow window;
    window.setWindowTitle("Rover HMI");
    window.setStyleSheet(QString("background: %1;").arg(theme::Bg));

    auto* tiling = new TilingContainer();

    // Scan the ament plugin index for all classes derived from GuiModule.
    // The base package "rover_hmi_core" is where the interface is declared;
    // concrete implementations can live in any package.
    pluginlib::ClassLoader<rover_hmi_core::GuiModule> loader(
        "rover_hmi_core", "rover_hmi_core::GuiModule");

    // Keep shared_ptrs alive for the lifetime of the window so that modules
    // outlive their widgets (pluginlib unloads the shared library when the
    // last shared_ptr is destroyed).
    std::vector<std::shared_ptr<rover_hmi_core::GuiModule>> modules;

    for (const auto& class_name : loader.getDeclaredClasses()) {
        try {
            auto module = loader.createSharedInstance(class_name);
            // Provide the ROS2 node before widget creation so that modules
            // can set up subscriptions/publishers in setNode().
            module->setNode(node);
            auto* widget = module->createWidget(tiling);
            tiling->addPanel(module->name(), widget, module->layoutHint(),
                             module->defaultVisible(), module->toggleCallback(),
                             module->keybindings(), module->sectionName());
            modules.push_back(module);
        } catch (const pluginlib::PluginlibException& ex) {
            RCLCPP_WARN(node->get_logger(),
                "Failed to load HMI plugin '%s': %s", class_name.c_str(), ex.what());
        }
    }
    RCLCPP_INFO(node->get_logger(), "Loaded %zu HMI modules", modules.size());

    // finalize() builds the initial dwindle tree from the accumulated panels
    // and must be called before the window is shown.
    tiling->finalize();
    window.setCentralWidget(tiling);
    window.resize(1600, 1000);
    window.showMaximized();

    // start() is called after the window is visible so modules can safely
    // start timers or subscriptions that trigger immediate repaints.
    for (auto& m : modules) m->start();

    int ret = app.exec();

    // stop() gives modules a chance to cancel timers and clean up before the
    // node is shut down and shared libraries are unloaded.
    for (auto& m : modules) m->stop();
    rclcpp::shutdown();
    return ret;
}
