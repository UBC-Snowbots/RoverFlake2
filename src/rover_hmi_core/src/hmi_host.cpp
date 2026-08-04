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
#include <QFile>
#include <QMainWindow>
#include <QShortcut>
#include <QTextStream>
#include <QTimer>

#include <algorithm>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>

// "A, B ,C" → {"A","B","C"} (trimmed, empties dropped)
static std::vector<std::string> splitPanelList(const std::string& csv) {
    std::vector<std::string> out;
    std::stringstream ss(csv);
    std::string item;
    while (std::getline(ss, item, ',')) {
        size_t a = item.find_first_not_of(" \t");
        if (a == std::string::npos) continue;
        size_t b = item.find_last_not_of(" \t");
        out.push_back(item.substr(a, b - a + 1));
    }
    return out;
}

// Font-scale cache: survives HMI restarts within a session, deleted on clean
// shutdown (and cleared by the OS on reboot) so a scale never becomes sticky.
static const char* kFontScaleCache = "/tmp/rover_hmi_fontscale";

// Application-wide filter: when a module rewrites a widget's stylesheet at
// runtime (button state changes), re-apply the current font scale to it.
class FontScaleFilter : public QObject {
public:
    bool eventFilter(QObject* obj, QEvent* ev) override {
        if (ev->type() == QEvent::StyleChange && theme::FontScale != 1.0)
            if (auto* w = qobject_cast<QWidget*>(obj))
                theme::rescaleWidget(w);
        return false;
    }
};

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
                             module->keybindings(), module->sectionName(),
                             [module]() { return module->saveState(); },
                             [module](const QJsonObject& st) { module->restoreState(st); });
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

    // Wall identity: instance names this window (title prefix + topic scope).
    const auto instance = node->declare_parameter<std::string>("instance", "");
    const QString title_base = instance.empty()
        ? QString("Rover HMI")
        : QString("Rover HMI [%1]").arg(QString::fromStdString(instance));
    window.setWindowTitle(title_base);

    // Window title follows the active layout / panel set.
    tiling->onLayoutChanged = [&window, title_base](const QString& name) {
        window.setWindowTitle(name.isEmpty() ? title_base
                                             : title_base + " — " + name);
    };

    // Known panel titles, for validating panel-set requests.
    std::vector<std::string> module_names;
    for (auto& m : modules) module_names.push_back(m->name());

    auto applyPanels = [&node, &module_names, tiling](const std::string& csv) {
        std::vector<std::string> known;
        for (const auto& t : splitPanelList(csv)) {
            if (std::find(module_names.begin(), module_names.end(), t)
                    != module_names.end())
                known.push_back(t);
            else
                RCLCPP_WARN(node->get_logger(),
                            "show_panels: unknown panel '%s'", t.c_str());
        }
        if (known.empty())
            RCLCPP_WARN(node->get_logger(), "show_panels: no valid panels in request");
        else
            tiling->showPanels(known);
    };
    auto applyLayout = [&node, tiling](const std::string& name) {
        if (tiling->loadLayoutByName(QString::fromStdString(name))) return;
        std::string names;
        for (const auto& e : tiling->layoutStore().list())
            names += (names.empty() ? "" : ", ") + e.name.toStdString();
        RCLCPP_WARN(node->get_logger(),
                    "load_layout: unknown layout '%s' (known: %s)",
                    name.c_str(), names.c_str());
    };

    // Startup selection: layout name wins over explicit panel list.
    const auto layout_param = node->declare_parameter<std::string>("layout", "");
    const auto panels_param = node->declare_parameter<std::string>("panels", "");
    if (!layout_param.empty())      applyLayout(layout_param);
    else if (!panels_param.empty()) applyPanels(panels_param);

    // Runtime control topics. Callbacks run on the Qt thread via the
    // spin_some timer, so they may touch widgets directly.
    // Volatile QoS: commands arriving while the HMI is down are dropped.
    auto load_layout_sub = node->create_subscription<std_msgs::msg::String>(
        "/hmi/load_layout", 10,
        [applyLayout](const std_msgs::msg::String& msg) { applyLayout(msg.data); });
    auto show_panels_sub = node->create_subscription<std_msgs::msg::String>(
        "/hmi/show_panels", 10,
        [applyPanels](const std_msgs::msg::String& msg) { applyPanels(msg.data); });

    // Per-instance control topics (wall mode): global topics above still apply
    // to every window; these address just this one.
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr inst_layout_sub, inst_panels_sub;
    if (!instance.empty()) {
        inst_layout_sub = node->create_subscription<std_msgs::msg::String>(
            "/hmi/" + instance + "/load_layout", 10,
            [applyLayout](const std_msgs::msg::String& msg) { applyLayout(msg.data); });
        inst_panels_sub = node->create_subscription<std_msgs::msg::String>(
            "/hmi/" + instance + "/show_panels", 10,
            [applyPanels](const std_msgs::msg::String& msg) { applyPanels(msg.data); });
    }

    window.setCentralWidget(tiling);
    window.resize(1600, 1000);
    window.showMaximized();

    // start() is called after the window is visible so modules can safely
    // start timers or subscriptions that trigger immediate repaints.
    for (auto& m : modules) m->start();

    // ── Runtime font zoom: Ctrl+= / Ctrl+- adjust, Ctrl+0 resets ────────────
    FontScaleFilter scale_filter;
    app.installEventFilter(&scale_filter);

    auto setScale = [&app](double s) {
        theme::applyFontScale(app, s);
        QFile f(kFontScaleCache);
        if (f.open(QIODevice::WriteOnly | QIODevice::Truncate))
            QTextStream(&f) << theme::FontScale;
    };
    const std::pair<QKeySequence, double> zoom_keys[] = {
        { QKeySequence::ZoomIn,       +0.1 },
        { QKeySequence("Ctrl+="),     +0.1 },
        { QKeySequence::ZoomOut,      -0.1 },
        { QKeySequence("Ctrl+0"),      0.0 },  // 0 = reset
    };
    for (const auto& [seq, delta] : zoom_keys) {
        auto* sc = new QShortcut(seq, &window);
        sc->setContext(Qt::ApplicationShortcut);
        QObject::connect(sc, &QShortcut::activated, [setScale, delta]() {
            setScale(delta == 0.0 ? 1.0 : theme::FontScale + delta);
        });
    }

    // Restore a scale left by a previous run this session, if any.
    {
        QFile f(kFontScaleCache);
        if (f.open(QIODevice::ReadOnly)) {
            double s = QTextStream(&f).readAll().trimmed().toDouble();
            if (s > 0.0 && s != 1.0) theme::applyFontScale(app, s);
        }
    }

    int ret = app.exec();
    QFile::remove(kFontScaleCache);

    // stop() gives modules a chance to cancel timers and clean up before the
    // node is shut down and shared libraries are unloaded.
    for (auto& m : modules) m->stop();
    rclcpp::shutdown();
    return ret;
}
