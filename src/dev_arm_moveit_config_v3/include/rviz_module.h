// rviz_module.h
//
// HMI plugin that embeds a live RViz viewport into the arm HMI sidebar.
//
// Lives in dev_arm_moveit_config_v3 because that is where all
// simulation/visualization config lives. Lazily launches RViz and
// robot_state_publisher as subprocesses when the user enables it in the
// sidebar. Uses xdotool to find the RViz window by title and embed it into
// the Qt widget container. The rviz config (arm_embedded.rviz) is also in
// this package's config directory.

#pragma once

#include <rover_hmi_core/gui_module.h>

#include <QWidget>
#include <QObject>
#include <QProcess>
#include <QTimer>
#include <QVBoxLayout>
#include <QLabel>
#include <string>
#include <functional>

class RvizModule : public QObject, public rover_hmi_core::GuiModule {
    Q_OBJECT
public:
    std::string name() const override { return "RViz Simulation"; }
    std::string layoutHint() const override { return "right"; }
    QWidget* createWidget(QWidget* parent) override;
    void start() override {}
    void stop() override;
    // Hidden by default so the heavy RViz process is not launched until the
    // user explicitly requests the visualization.
    bool defaultVisible() const override { return false; }
    std::function<void(bool)> toggleCallback() override;

    void onToggled(bool visible);

private:
    // Spawns robot_state_publisher and rviz2 as child QProcesses, then starts
    // the embed_timer_ polling loop to capture the RViz window once ready.
    void launchProcesses();
    // Called on each embed_timer_ tick to check whether the RViz window has
    // appeared yet.
    void tryEmbed();
    // Wraps the native RViz window in a Qt container widget and inserts it
    // into the module layout.
    void embedWindow();
    // Removes the embedded Qt container and minimizes the detached RViz
    // window so it does not clutter the desktop.
    void unembed();

    QWidget* container_ = nullptr;
    QVBoxLayout* layout_ = nullptr;
    QLabel* status_label_ = nullptr;
    QWidget* embedded_widget_ = nullptr;

    QProcess* rsp_process_ = nullptr;
    QProcess* rviz_process_ = nullptr;
    // Periodic timer used to poll xdotool for the RViz window ID. RViz does
    // not emit a signal when its window is ready, so polling is necessary.
    QTimer* embed_timer_ = nullptr;
    int embed_attempts_ = 0;

    std::string robot_description_;
    std::string params_file_;
    unsigned long rviz_wid_ = 0;
    bool embedded_ = false;
    bool launched_ = false;
};
