#include "gui_module.h"
#include "catppuccin.h"
#include "moteus_data_bus.h"
#include "motor_status_module.h"
#include "plotting_module.h"
#include "send_command_module.h"
#include "command_log_module.h"
#include "motor_config_module.h"
#include "tiling_container.h"

#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

struct ModuleEntry {
    std::unique_ptr<GuiModule> module;
    std::string layout_hint;
};

int main(int argc, char* argv[]) {
    // Init ROS2 first (consumes ROS args like --ros-args)
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rover_hmi_v2");

    QApplication app(argc, argv);
    theme::applyGlobalStylesheet(app);

    QMainWindow window;
    window.setWindowTitle("Rover HMI v2");
    window.setStyleSheet(QString("background: %1;").arg(theme::Bg));

    // Shared data bus — talks to driver via ROS2 topics
    auto* bus = new MoteusDataBus(node, &window);

    // -----------------------------------------------------------------------
    // Register modules here
    //   "main"    = top-left (large)
    //   "right"   = top-right (stacked vertically)
    //   "bottom"  = bottom strip
    // -----------------------------------------------------------------------
    std::vector<ModuleEntry> entries;

    entries.push_back({std::make_unique<MotorStatusModule>(),  "main"});
    entries.push_back({std::make_unique<MotorConfigModule>(), "main"});
    entries.push_back({std::make_unique<PlottingModule>(),    "right"});
    entries.push_back({std::make_unique<SendCommandModule>(), "right"});
    entries.push_back({std::make_unique<CommandLogModule>(),  "bottom"});

    // -----------------------------------------------------------------------
    // Build tiling layout
    // -----------------------------------------------------------------------
    auto* tiling = new TilingContainer();

    for (auto& entry : entries) {
        entry.module->setDataBus(bus);
        auto* widget = entry.module->createWidget(tiling);
        tiling->addPanel(entry.module->name(), widget, entry.layout_hint);
    }

    tiling->finalize();
    window.setCentralWidget(tiling);

    window.resize(1600, 1000);
    window.showMaximized();

    for (auto& entry : entries)
        entry.module->start();
    bus->start();

    int ret = app.exec();

    bus->stop();
    for (auto& entry : entries)
        entry.module->stop();

    rclcpp::shutdown();
    return ret;
}
