// rviz_module.cpp
//
// HMI plugin that embeds a live RViz viewport into the arm HMI sidebar.
//
// Lives in dev_arm_moveit_config_v3 because that is where all
// simulation/visualization config lives. Lazily launches RViz and
// robot_state_publisher as subprocesses when the user enables it in the
// sidebar. Uses xdotool to find the RViz window by title and embed it into
// the Qt widget container. The rviz config (arm_embedded.rviz) is also in
// this package's config directory.

#include "rviz_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QWindow>
#include <QTemporaryFile>
#include <QDir>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>

#include <pluginlib/class_list_macros.hpp>

QWidget* RvizModule::createWidget(QWidget* parent) {
    container_ = new QWidget(parent);
    layout_ = new QVBoxLayout(container_);
    layout_->setContentsMargins(0, 0, 0, 0);

    status_label_ = new QLabel("Toggle in sidebar to launch RViz", container_);
    status_label_->setAlignment(Qt::AlignCenter);
    status_label_->setStyleSheet(
        QString("color: %1; font-size: 14px;").arg(theme::Text));
    layout_->addWidget(status_label_);

    try {
        std::string urdf_path =
            ament_index_cpp::get_package_share_directory("dev_arm_description_v2")
            + "/urdf/dev_arm.urdf";
        std::ifstream f(urdf_path);
        if (f.is_open()) {
            std::ostringstream ss;
            ss << f.rdbuf();
            robot_description_ = ss.str();
        }
    } catch (...) {}

    return container_;
}

std::function<void(bool)> RvizModule::toggleCallback() {
    return [this](bool visible) { onToggled(visible); };
}

void RvizModule::onToggled(bool visible) {
    if (visible) {
        // Lazy launch: RViz is heavy and the user may never need it, so we
        // defer spawning the processes until the first time the panel is
        // enabled. Subsequent toggles re-embed the already-running window
        // instead of spawning a second instance.
        if (!launched_) {
            launchProcesses();
        } else if (rviz_wid_ != 0 && !embedded_) {
            embedWindow();
        }
    } else {
        if (embedded_) unembed();
    }
}

void RvizModule::launchProcesses() {
    if (robot_description_.empty()) {
        status_label_->setText("Error: Could not load URDF");
        return;
    }

    launched_ = true;
    status_label_->setText("Starting RViz...");
    status_label_->show();

    params_file_ = QDir::tempPath().toStdString() + "/rover_hmi_rsp_params.yaml";
    {
        std::ofstream pf(params_file_);
        pf << "robot_state_publisher:\n"
           << "  ros__parameters:\n"
           << "    robot_description: |\n";
        std::istringstream iss(robot_description_);
        std::string line;
        while (std::getline(iss, line))
            pf << "      " << line << "\n";
    }

    rsp_process_ = new QProcess(container_);
    rsp_process_->setProgram("ros2");
    rsp_process_->setArguments({
        "run", "robot_state_publisher", "robot_state_publisher",
        "--ros-args", "--params-file", QString::fromStdString(params_file_)
    });
    rsp_process_->start();

    std::string rviz_config;
    try {
        rviz_config = ament_index_cpp::get_package_share_directory("dev_arm_moveit_config_v3")
                      + "/config/arm_embedded.rviz";
    } catch (...) {}

    rviz_process_ = new QProcess(container_);
    rviz_process_->setProgram("rviz2");
    QStringList args;
    if (!rviz_config.empty())
        args << "-d" << QString::fromStdString(rviz_config);
    rviz_process_->setArguments(args);
    rviz_process_->start();

    // Poll every 500 ms for the RViz window to appear. RViz does not signal
    // when its window is ready, so we use xdotool to search by window title
    // on each tick. embed_attempts_ caps the poll loop so we don't wait
    // forever if RViz fails to open a visible window.
    embed_attempts_ = 0;
    embed_timer_ = new QTimer(this);
    connect(embed_timer_, &QTimer::timeout, this, &RvizModule::tryEmbed);
    embed_timer_->start(500);
}

void RvizModule::tryEmbed() {
    embed_attempts_++;

    if (!rviz_process_ || rviz_process_->state() == QProcess::NotRunning) {
        status_label_->setText("RViz failed to start");
        embed_timer_->stop();
        return;
    }

    if (embed_attempts_ > 40) {
        status_label_->setText("RViz running (window not captured)");
        embed_timer_->stop();
        return;
    }

    QProcess finder;
    finder.start("xdotool", {"search", "--name", "RViz"});
    finder.waitForFinished(2000);

    QString output = finder.readAllStandardOutput().trimmed();
    if (output.isEmpty()) return;

    QStringList wids = output.split('\n');
    bool ok = false;
    unsigned long wid = wids.last().toULong(&ok);
    if (!ok || wid == 0) return;

    embed_timer_->stop();
    rviz_wid_ = wid;
    embedWindow();
}

void RvizModule::embedWindow() {
    if (rviz_wid_ == 0 || embedded_) return;

    QWindow* foreign = QWindow::fromWinId(rviz_wid_);
    if (!foreign) {
        status_label_->setText("Failed to capture RViz window");
        return;
    }

    embedded_widget_ = QWidget::createWindowContainer(foreign, container_);
    embedded_widget_->setMinimumSize(200, 200);
    embedded_widget_->setFocusPolicy(Qt::NoFocus);

    status_label_->hide();
    layout_->addWidget(embedded_widget_);
    embedded_ = true;
}

void RvizModule::unembed() {
    if (!embedded_ || !embedded_widget_) return;

    layout_->removeWidget(embedded_widget_);
    embedded_widget_->deleteLater();
    embedded_widget_ = nullptr;
    embedded_ = false;

    // When the panel is hidden we release the Qt window container but keep
    // the RViz process alive so re-enabling is fast. The detached native
    // window is minimized via xdotool so it does not linger on the desktop.
    if (rviz_wid_ != 0)
        QProcess::execute("xdotool", {"windowminimize", QString::number(rviz_wid_)});

    status_label_->setText("RViz running (hidden)");
    status_label_->show();
}

void RvizModule::stop() {
    if (embed_timer_) {
        embed_timer_->stop();
        embed_timer_->deleteLater();
        embed_timer_ = nullptr;
    }

    if (embedded_widget_) {
        embedded_widget_->deleteLater();
        embedded_widget_ = nullptr;
    }

    if (rviz_process_ && rviz_process_->state() != QProcess::NotRunning) {
        rviz_process_->terminate();
        if (!rviz_process_->waitForFinished(3000))
            rviz_process_->kill();
    }

    if (rsp_process_ && rsp_process_->state() != QProcess::NotRunning) {
        rsp_process_->terminate();
        if (!rsp_process_->waitForFinished(3000))
            rsp_process_->kill();
    }

    if (!params_file_.empty())
        std::remove(params_file_.c_str());
}

PLUGINLIB_EXPORT_CLASS(RvizModule, rover_hmi_core::GuiModule)
