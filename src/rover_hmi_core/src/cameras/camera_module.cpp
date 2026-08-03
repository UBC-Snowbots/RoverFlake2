#include "camera_module.h"
#include "camera_map.h"
#include "camera_viewport.h"

#include <QHBoxLayout>
#include <QShortcut>
#include <QSplitter>

#include <pluginlib/class_list_macros.hpp>

using rover_hmi_core::camera_config::topicFor;

QWidget* CameraModule::createWidget(QWidget* parent)
{
    auto* widget = new QWidget(parent);
    widget->setFocusPolicy(Qt::ClickFocus);
    auto* row = new QHBoxLayout(widget);
    row->setContentsMargins(0, 0, 0, 0);

    QString err;
    cams_ = rover_hmi_core::camera_config::load(&err);

    viewport_ = new CameraViewport(widget);
    map_ = new CameraMap(cams_, widget);
    map_->onSelect = [this](int idx) { switchTo(idx); };

    auto* split = new QSplitter(Qt::Horizontal, widget);
    split->addWidget(viewport_);
    split->addWidget(map_);
    split->setStretchFactor(0, 3);
    split->setStretchFactor(1, 1);
    row->addWidget(split);

    if (cams_.empty()) {
        viewport_->setError(QStringLiteral("camera_map.json: %1").arg(err));
        return widget;
    }

    // Plain 1..N, active only while this tile has focus (Alt+1..9 belongs
    // to the host's sidebar toggles).
    for (int i = 0; i < int(cams_.size()) && i < 9; ++i) {
        auto* sc = new QShortcut(QKeySequence(QString::number(i + 1)), widget);
        sc->setContext(Qt::WidgetWithChildrenShortcut);
        QObject::connect(sc, &QShortcut::activated, [this, i]() { switchTo(i); });
    }

    liveness_timer_ = new QTimer(widget);
    QObject::connect(liveness_timer_, &QTimer::timeout,
                     [this]() { onLivenessTick(); });
    return widget;
}

void CameraModule::setNode(rclcpp::Node::SharedPtr node) { node_ = node; }

void CameraModule::start()
{
    if (!cams_.empty() && visible_) switchTo(0);
    if (liveness_timer_) liveness_timer_->start(500);
}

void CameraModule::onVisibility(bool on)
{
    visible_ = on;
    if (!on) {
        sub_.reset();
        if (viewport_) viewport_->setNoSignal();
        return;
    }
    // sub_ is null here, so switchTo re-subscribes even to the same index.
    if (!cams_.empty() && node_) switchTo(active_ >= 0 ? active_ : 0);
}

void CameraModule::stop()
{
    if (liveness_timer_) liveness_timer_->stop();
    sub_.reset();
}

void CameraModule::switchTo(int idx)
{
    if (idx < 0 || idx >= int(cams_.size()) || !node_) return;
    if (idx == active_ && sub_) return;
    active_ = idx;
    sub_.reset();
    last_frame_.invalidate();
    viewport_->setLabel(cams_[size_t(idx)].label);
    viewport_->setNoSignal();
    map_->setActive(idx);

    // depth-1 best-effort: DDS drops stale frames instead of queueing them
    // behind the 20 ms spin_some() pump.
    sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        topicFor(cams_[size_t(idx)]).toStdString(),
        rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
            if (last_frame_.isValid()) last_frame_.restart();
            else                       last_frame_.start();
            viewport_->setFrame(std::move(msg));
        });
}

void CameraModule::onLivenessTick()
{
    if (!node_ || cams_.empty()) return;
    std::vector<bool> alive(cams_.size(), false);
    for (size_t i = 0; i < cams_.size(); ++i)
        alive[i] = node_->count_publishers(topicFor(cams_[i]).toStdString()) > 0;
    map_->setAlive(alive);

    if (viewport_->hasFrame() && last_frame_.isValid()
        && last_frame_.elapsed() > 1000)
        viewport_->setNoSignal();
}

PLUGINLIB_EXPORT_CLASS(CameraModule, rover_hmi_core::GuiModule)
