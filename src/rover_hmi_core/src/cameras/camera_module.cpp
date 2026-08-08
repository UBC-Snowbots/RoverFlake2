#include "camera_module.h"
#include "camera_grid.h"
#include "camera_viewport.h"

#include <QHBoxLayout>
#include <QJsonArray>
#include <QPushButton>
#include <QSet>
#include <QShortcut>
#include <QVBoxLayout>
#include <algorithm>

#include <rover_hmi_core/catppuccin.h>

#include <pluginlib/class_list_macros.hpp>

using rover_hmi_core::camera_config::topicFor;

QWidget* CameraModule::createWidget(QWidget* parent)
{
    auto* widget = new QWidget(parent);
    widget->setFocusPolicy(Qt::ClickFocus);
    auto* lay = new QVBoxLayout(widget);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->setSpacing(2);

    QString err;
    cams_ = rover_hmi_core::camera_config::load(&err);

    if (cams_.empty()) {
        auto* error_view = new CameraViewport(widget);
        error_view->setPlaceholder(QPixmap(rover_hmi_core::camera_config::catError()));
        error_view->setError(QStringLiteral("camera_map.json: %1").arg(err));
        lay->addWidget(error_view, 1);
        return widget;
    }

    grid_widget_ = new CameraGrid(widget);
    QStringList cell_names;
    for (int i = 0; i < int(cams_.size()); ++i) {
        auto* cell = new CameraViewport(grid_widget_);
        cell->setMinimumSize(0, 0);  // geometry is tree-managed, not layout-managed
        cell->setLabel(cams_[size_t(i)].label);
        cell->setPlaceholder(QPixmap(rover_hmi_core::camera_config::catFor(i)));
        cells_.push_back(cell);
        cell_names << cams_[size_t(i)].name;
    }
    grid_widget_->setCells(cells_, cell_names);
    in_grid_.assign(cams_.size(), true);
    lay->addWidget(grid_widget_, 1);

    // Bottom selector: one toggle button per camera controls grid membership.
    auto* bar = new QWidget(widget);
    auto* bar_lay = new QHBoxLayout(bar);
    bar_lay->setContentsMargins(8, 0, 8, 4);
    bar_lay->setSpacing(6);
    for (int i = 0; i < int(cams_.size()); ++i) {
        auto* btn = new QPushButton(
            QStringLiteral("%1 %2").arg(i + 1).arg(cams_[size_t(i)].label), bar);
        btn->setCheckable(true);
        btn->setChecked(true);
        btn->setFocusPolicy(Qt::NoFocus);  // keys stay with the module
        QObject::connect(btn, &QPushButton::clicked,
                         [this, i]() { toggleInGrid(i); });
        buttons_.push_back(btn);
        bar_lay->addWidget(btn, 1);
    }
    lay->addWidget(bar);
    rebuildGrid();

    subs_.resize(cams_.size());
    last_frames_.resize(cams_.size());

    // Module-focused shortcuts only (Alt+1..9 belongs to the host sidebar).
    for (int i = 0; i < int(cams_.size()) && i < 9; ++i) {
        auto* sc = new QShortcut(QKeySequence(QString::number(i + 1)), widget);
        sc->setContext(Qt::WidgetWithChildrenShortcut);
        QObject::connect(sc, &QShortcut::activated,
                         [this, i]() { toggleInGrid(i); });
    }

    liveness_timer_ = new QTimer(widget);
    QObject::connect(liveness_timer_, &QTimer::timeout,
                     [this]() { onLivenessTick(); });
    return widget;
}

void CameraModule::setNode(rclcpp::Node::SharedPtr node) { node_ = node; }

void CameraModule::start()
{
    if (visible_) resubscribe();
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
        for (auto* cell : cells_) cell->setNoSignal();
        return;
    }
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
    for (int i = 0; i < int(cams_.size()); ++i) {
        if (!in_grid_[size_t(i)]) continue;
        last_frames_[size_t(i)].invalidate();
        cells_[size_t(i)]->setNoSignal();
        subs_[size_t(i)] = makeSub(i, cells_[size_t(i)]);
    }
}

void CameraModule::toggleInGrid(int idx)
{
    if (idx < 0 || idx >= int(in_grid_.size())) return;
    if (in_grid_[size_t(idx)]
        && std::count(in_grid_.begin(), in_grid_.end(), true) <= 1) {
        rebuildGrid();  // undo the button's own toggle
        return;         // never empty the grid
    }
    in_grid_[size_t(idx)] = !in_grid_[size_t(idx)];
    rebuildGrid();
    resubscribe();
}

void CameraModule::rebuildGrid()
{
    if (!grid_widget_) return;
    grid_widget_->setMembership(in_grid_);
    for (size_t i = 0; i < buttons_.size(); ++i) {
        auto* btn = buttons_[i];
        btn->setChecked(in_grid_[i]);
        btn->setStyleSheet(QStringLiteral(
            "QPushButton { color: %1; border-color: %1; }")
            .arg(in_grid_[i] ? theme::Green : theme::TextDim));
    }
}

bool CameraModule::gridOp(TilingOp op, int dx, int dy)
{
    return grid_widget_ && grid_widget_->handleOp(op, dx, dy);
}

void CameraModule::onLivenessTick()
{
    // Hold the last frame through short dropouts; only admit NO SIGNAL after 5 s.
    for (size_t i = 0; i < cells_.size(); ++i)
        if (cells_[i]->hasFrame() && last_frames_[i].isValid()
            && last_frames_[i].elapsed() > 5000)
            cells_[i]->setNoSignal();
}

QJsonObject CameraModule::saveState() const
{
    if (cams_.empty()) return {};
    QJsonObject st;
    QJsonArray members;
    for (size_t i = 0; i < cams_.size(); ++i)
        if (in_grid_[i]) members.append(cams_[i].name);
    st["grid_cams"] = members;
    if (grid_widget_) {
        QJsonObject tree = grid_widget_->saveTree();
        if (!tree.isEmpty()) st["grid_tree"] = tree;
    }
    return st;
}

void CameraModule::restoreState(const QJsonObject& st)
{
    if (cams_.empty()) return;
    if (st.contains(QStringLiteral("grid_cams"))) {
        QSet<QString> members;
        for (auto v : st["grid_cams"].toArray()) members.insert(v.toString());
        if (!members.isEmpty())
            for (size_t i = 0; i < cams_.size(); ++i)
                in_grid_[i] = members.contains(cams_[i].name);
    }
    rebuildGrid();
    // Cell arrangement rides on top of membership; older layouts without a
    // saved tree keep the membership-order dwindle from rebuildGrid().
    if (grid_widget_ && st.contains(QStringLiteral("grid_tree")))
        grid_widget_->restoreTree(st["grid_tree"].toObject());
    resubscribe();
}

PLUGINLIB_EXPORT_CLASS(CameraModule, rover_hmi_core::GuiModule)
