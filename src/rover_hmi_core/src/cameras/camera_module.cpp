#include "camera_module.h"
#include "camera_list_overlay.h"
#include "camera_viewport.h"

#include <QGridLayout>
#include <QLabel>
#include <QShortcut>
#include <QStackedWidget>
#include <QVBoxLayout>

#include <rover_hmi_core/catppuccin.h>

#include <pluginlib/class_list_macros.hpp>

using rover_hmi_core::camera_config::topicFor;

QWidget* CameraModule::createWidget(QWidget* parent)
{
    auto* widget = new QWidget(parent);
    widget->setFocusPolicy(Qt::ClickFocus);
    auto* lay = new QVBoxLayout(widget);
    lay->setContentsMargins(0, 0, 0, 0);

    QString err;
    cams_ = rover_hmi_core::camera_config::load(&err);

    stack_ = new QStackedWidget(widget);
    lay->addWidget(stack_);

    viewport_ = new CameraViewport(stack_);
    stack_->addWidget(viewport_);

    auto* grid_page = new QWidget(stack_);
    auto* grid_col = new QVBoxLayout(grid_page);
    grid_col->setContentsMargins(0, 0, 0, 0);
    grid_col->setSpacing(2);
    auto* grid_cells = new QWidget(grid_page);
    auto* grid = new QGridLayout(grid_cells);
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setSpacing(4);
    for (int i = 0; i < int(cams_.size()); ++i) {
        auto* cell = new CameraViewport(grid_cells);
        cell->setMinimumSize(160, 120);
        cell->setLabel(cams_[size_t(i)].label);
        cell->onClick = [this, i]() { setGrid(false); switchTo(i); };
        grid->addWidget(cell, i / 2, i % 2);
        cells_.push_back(cell);
    }
    grid_col->addWidget(grid_cells, 1);
    auto* grid_hint = new QLabel(
        QStringLiteral("G single view   ·   click a feed to zoom in"), grid_page);
    grid_hint->setStyleSheet(QStringLiteral("color: %1; padding: 2px 8px;")
                                 .arg(theme::TextDim));
    grid_hint->setAlignment(Qt::AlignRight);
    grid_col->addWidget(grid_hint);
    stack_->addWidget(grid_page);

    std::vector<QString> labels;
    for (const auto& c : cams_) labels.push_back(c.label);
    list_ = new CameraListOverlay(labels, viewport_);
    list_->raise();

    if (cams_.empty()) {
        viewport_->setError(QStringLiteral("camera_map.json: %1").arg(err));
        return widget;
    }

    subs_.resize(cams_.size());
    last_frames_.resize(cams_.size());

    // Module-focused shortcuts only (Alt+1..9 belongs to the host sidebar).
    auto bind = [widget](const QKeySequence& seq, std::function<void()> fn) {
        auto* sc = new QShortcut(seq, widget);
        sc->setContext(Qt::WidgetWithChildrenShortcut);
        QObject::connect(sc, &QShortcut::activated, fn);
    };
    for (int i = 0; i < int(cams_.size()) && i < 9; ++i)
        bind(QKeySequence(QString::number(i + 1)),
             [this, i]() { setGrid(false); switchTo(i); });
    const int n = int(cams_.size());
    bind(QKeySequence(Qt::Key_Up),
         [this, n]() { if (!grid_) switchTo((active_ + n - 1) % n); });
    bind(QKeySequence(Qt::Key_Down),
         [this, n]() { if (!grid_) switchTo((active_ + 1) % n); });
    bind(QKeySequence(Qt::Key_G), [this]() { setGrid(!grid_); });

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

void CameraModule::stop()
{
    if (liveness_timer_) liveness_timer_->stop();
    unsubscribeAll();
}

void CameraModule::onVisibility(bool on)
{
    visible_ = on;
    if (!on) {
        unsubscribeAll();
        if (viewport_) viewport_->setNoSignal();
        for (auto* cell : cells_) cell->setNoSignal();
        return;
    }
    if (!cams_.empty() && node_ && active_ < 0) active_ = 0;
    resubscribe();
}

void CameraModule::unsubscribeAll()
{
    for (auto& s : subs_) s.reset();
}

rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
CameraModule::makeSub(int idx, CameraViewport* target)
{
    // Reliable depth-5 (matches rviz): backlog bounded to ~5 frames so the
    // view can never drift more than ~165 ms behind live.
    return node_->create_subscription<sensor_msgs::msg::Image>(
        topicFor(cams_[size_t(idx)]).toStdString(),
        rclcpp::QoS(5),
        [this, idx, target](sensor_msgs::msg::Image::ConstSharedPtr msg) {
            auto& clk = last_frames_[size_t(idx)];
            if (clk.isValid()) clk.restart();
            else               clk.start();
            target->setFrame(std::move(msg));
        });
}

void CameraModule::resubscribe()
{
    unsubscribeAll();
    if (!visible_ || !node_ || cams_.empty()) return;
    if (grid_) {
        for (int i = 0; i < int(cams_.size()); ++i) {
            last_frames_[size_t(i)].invalidate();
            cells_[size_t(i)]->setNoSignal();
            subs_[size_t(i)] = makeSub(i, cells_[size_t(i)]);
        }
    } else if (active_ >= 0) {
        last_frames_[size_t(active_)].invalidate();
        viewport_->setLabel(cams_[size_t(active_)].label);
        viewport_->setNoSignal();
        list_->setActive(active_);
        subs_[size_t(active_)] = makeSub(active_, viewport_);
    }
}

void CameraModule::switchTo(int idx)
{
    if (idx < 0 || idx >= int(cams_.size()) || !node_ || grid_) return;
    if (idx == active_ && subs_[size_t(idx)]) return;
    active_ = idx;
    resubscribe();
}

void CameraModule::setGrid(bool on)
{
    if (grid_ == on || cams_.empty() || !stack_) return;
    grid_ = on;
    stack_->setCurrentIndex(on ? 1 : 0);
    resubscribe();
}

void CameraModule::onLivenessTick()
{
    if (!node_ || cams_.empty()) return;
    std::vector<bool> alive(cams_.size(), false);
    for (size_t i = 0; i < cams_.size(); ++i)
        alive[i] = node_->count_publishers(topicFor(cams_[i]).toStdString()) > 0;
    list_->setAlive(alive);

    // Hold the last frame through short dropouts; only admit NO SIGNAL after 5 s.
    auto stale = [this](size_t i) {
        return last_frames_[i].isValid() && last_frames_[i].elapsed() > 5000;
    };
    if (grid_) {
        for (size_t i = 0; i < cells_.size(); ++i)
            if (cells_[i]->hasFrame() && stale(i)) cells_[i]->setNoSignal();
    } else if (active_ >= 0 && viewport_->hasFrame() && stale(size_t(active_))) {
        viewport_->setNoSignal();
    }
}

PLUGINLIB_EXPORT_CLASS(CameraModule, rover_hmi_core::GuiModule)
