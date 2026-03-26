#include "gui_module.h"
#include "catppuccin.h"
#include "moteus_data_bus.h"
#include "motor_status_module.h"
#include "plotting_module.h"
#include "send_command_module.h"
#include "tiling_container.h"

#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>

#include <memory>
#include <vector>

struct ModuleEntry {
    std::unique_ptr<GuiModule> module;
    std::string layout_hint;  // "main", "right", "bottom"
};

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    theme::applyGlobalStylesheet(app);

    QMainWindow window;
    window.setWindowTitle("Rover HMI v2");
    window.setStyleSheet(QString("background: %1;").arg(theme::Bg));

    // Shared data bus
    auto* bus = new MoteusDataBus(&window);

    // -----------------------------------------------------------------------
    // Register modules here
    // -----------------------------------------------------------------------
    std::vector<ModuleEntry> entries;

    entries.push_back({std::make_unique<MotorStatusModule>(), "main"});
    entries.push_back({std::make_unique<PlottingModule>(),    "right"});
    entries.push_back({std::make_unique<SendCommandModule>(), "right"});

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

    // Keybinding hint in title
    window.setWindowTitle("Rover HMI v2  |  Tab: cycle focus  |  Super+Arrow: navigate  |  Super+Shift+Arrow: swap");

    window.resize(1500, 900);
    window.showMaximized();

    for (auto& entry : entries)
        entry.module->start();
    bus->start();

    int ret = app.exec();

    bus->stop();
    for (auto& entry : entries)
        entry.module->stop();

    return ret;
}
