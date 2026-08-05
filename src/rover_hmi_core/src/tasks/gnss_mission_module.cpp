// gnss_mission_module.cpp — "GNSS Mission"

#include "rover_hmi_core/tasks/gnss_mission_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QFileInfo>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace {

// mission_manager rejects tags on a fix older than this (stale_fix_sec).
constexpr double kStaleSec = 2.0;

QPushButton* button(const QString& text, const QString& color) {
    auto* b = new QPushButton(text);
    b->setStyleSheet(QString(
        "QPushButton { color: %1; background: %2; border: 1px solid %3;"
        "  border-radius: 4px; padding: 6px 10px; font-weight: bold; }"
        "QPushButton:pressed { background: #1a1a1a; }")
        .arg(color, theme::BgPanel, theme::BorderDim));
    return b;
}

}  // namespace

void GnssMissionModule::setNode(rclcpp::Node::SharedPtr node) {
    node_ = node;
    fix_stamp_ = rclcpp::Time(0, 0, node->get_clock()->get_clock_type());
    fix_sub_ = node->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gnss_fix", 10, [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            fix_ = msg;
            fix_stamp_ = node_->now();
        });
    start_cli_     = node->create_client<StartMission>("/mission/start");
    stop_cli_      = node->create_client<Trigger>("/mission/stop");
    tag_cli_       = node->create_client<TagPoint>("/tag_point");
    seg_start_cli_ = node->create_client<Segment>("/segment_start");
    seg_end_cli_   = node->create_client<Segment>("/segment_end");
}

QWidget* GnssMissionModule::createWidget(QWidget* parent) {
    auto* root = new QWidget(parent);
    root->setStyleSheet(QString("background: %1; color: %2;")
                            .arg(theme::Bg, theme::Text));
    auto* col = new QVBoxLayout(root);
    col->setSpacing(8);
    col->setContentsMargins(12, 12, 12, 12);

    const QString lineCss = QString(
        "QLineEdit { color: %1; background: %2; border: 1px solid %3;"
        "  border-radius: 4px; padding: 6px; }")
        .arg(theme::Text, theme::BgPanel, theme::BorderDim);

    fix_label_ = new QLabel();
    fix_label_->setFont(QFont("monospace", theme::FontSize, QFont::Bold));
    col->addWidget(fix_label_);

    auto* mission_row = new QHBoxLayout();
    mission_name_ = new QLineEdit();
    mission_name_->setPlaceholderText("mission name");
    mission_name_->setStyleSheet(lineCss);
    mission_row->addWidget(mission_name_, 1);
    auto* start = button("Start", theme::Green);
    QObject::connect(start, &QPushButton::clicked, [this]() { startMission(); });
    mission_row->addWidget(start);
    auto* stop = button("Stop", theme::Red);
    QObject::connect(stop, &QPushButton::clicked, [this]() { stopMission(); });
    mission_row->addWidget(stop);
    mission_label_ = new QLabel("—");
    mission_label_->setFont(QFont("monospace", theme::FontSize));
    mission_label_->setStyleSheet(QString("color:%1;").arg(theme::TextDim));
    mission_row->addWidget(mission_label_);
    col->addLayout(mission_row);

    auto* tag_row = new QHBoxLayout();
    tag_label_ = new QLineEdit();
    tag_label_->setPlaceholderText("tag label (optional)");
    tag_label_->setStyleSheet(lineCss);
    tag_row->addWidget(tag_label_, 1);
    // Categories accepted by /tag_point (mission_io.CATEGORIES).
    const struct { const char* cat; const char* color; } cats[] = {
        {"start", theme::Yellow}, {"site", theme::Green},
        {"sample", theme::Cyan},  {"obstacle", theme::Red},
        {"landmark", theme::Text}};
    for (const auto& c : cats) {
        auto* b = button(c.cat, c.color);
        const QString cat = c.cat;
        QObject::connect(b, &QPushButton::clicked, [this, cat]() { tag(cat); });
        tag_row->addWidget(b);
    }
    col->addLayout(tag_row);

    auto* seg_row = new QHBoxLayout();
    seg_name_ = new QLineEdit();
    seg_name_->setPlaceholderText("segment name");
    seg_name_->setStyleSheet(lineCss);
    seg_row->addWidget(seg_name_, 1);
    auto* seg_open = button("Seg start", theme::Green);
    QObject::connect(seg_open, &QPushButton::clicked, [this]() { segment(true); });
    seg_row->addWidget(seg_open);
    auto* seg_close = button("Seg end", theme::Yellow);
    QObject::connect(seg_close, &QPushButton::clicked, [this]() { segment(false); });
    seg_row->addWidget(seg_close);
    col->addLayout(seg_row);

    status_ = new QLabel();
    status_->setFont(QFont("monospace", theme::FontSize - 2));
    status_->setWordWrap(true);
    col->addWidget(status_);
    col->addStretch();

    auto* timer = new QTimer(root);
    QObject::connect(timer, &QTimer::timeout, [this]() { refreshFix(); });
    timer->start(500);
    refreshFix();
    return root;
}

// Fix callbacks and service responses arrive on the Qt main thread (the host
// pumps spin_some from a QTimer), so widgets are touched directly.
void GnssMissionModule::refreshFix() {
    if (!fix_) {
        fix_label_->setText("FIX  none");
        fix_label_->setStyleSheet(QString("color:%1;").arg(theme::Red));
        return;
    }
    const double age = (node_->now() - fix_stamp_).seconds();
    const bool stale = age > kStaleSec;
    const bool no_fix = fix_->status.status < 0;  // reader repeats last known
    fix_label_->setText(QString("FIX  %1, %2 · %3s%4")
                            .arg(fix_->latitude, 0, 'f', 6)
                            .arg(fix_->longitude, 0, 'f', 6)
                            .arg(age, 0, 'f', 1)
                            .arg(no_fix ? "  (last known — no fix)" : ""));
    fix_label_->setStyleSheet(QString("color:%1;").arg(
        stale ? theme::Red : no_fix ? theme::Yellow : theme::Green));
}

void GnssMissionModule::report(bool ok, const QString& msg) {
    status_->setText(msg);
    status_->setStyleSheet(QString("color:%1;").arg(ok ? theme::Green : theme::Red));
}

void GnssMissionModule::startMission() {
    if (!start_cli_->service_is_ready())
        return report(false, "mission_manager not running");
    auto req = std::make_shared<StartMission::Request>();
    req->name = mission_name_->text().trimmed().toStdString();
    start_cli_->async_send_request(req, [this](rclcpp::Client<StartMission>::SharedFuture f) {
        const auto r = f.get();
        report(r->ok, QString::fromStdString(r->message));
        if (r->ok) {
            mission_label_->setText(QFileInfo(QString::fromStdString(r->mission_dir)).fileName());
            mission_label_->setStyleSheet(QString("color:%1;").arg(theme::Green));
        }
    });
}

void GnssMissionModule::stopMission() {
    if (!stop_cli_->service_is_ready())
        return report(false, "mission_manager not running");
    stop_cli_->async_send_request(std::make_shared<Trigger::Request>(),
                                  [this](rclcpp::Client<Trigger>::SharedFuture f) {
        const auto r = f.get();
        report(r->success, QString::fromStdString(r->message));
        if (r->success) {
            mission_label_->setText("—");
            mission_label_->setStyleSheet(QString("color:%1;").arg(theme::TextDim));
        }
    });
}

void GnssMissionModule::tag(const QString& category) {
    if (!tag_cli_->service_is_ready())
        return report(false, "mission_manager not running");
    auto req = std::make_shared<TagPoint::Request>();
    req->category = category.toStdString();
    req->label = tag_label_->text().trimmed().toStdString();
    tag_cli_->async_send_request(req, [this](rclcpp::Client<TagPoint>::SharedFuture f) {
        const auto r = f.get();
        report(r->ok, r->ok ? QString("%1 @ %2, %3")
                                  .arg(QString::fromStdString(r->id))
                                  .arg(r->lat, 0, 'f', 6)
                                  .arg(r->lon, 0, 'f', 6)
                            : QString::fromStdString(r->message));
        if (r->ok) tag_label_->clear();
    });
}

void GnssMissionModule::segment(bool open) {
    auto& cli = open ? seg_start_cli_ : seg_end_cli_;
    if (!cli->service_is_ready())
        return report(false, "mission_manager not running");
    auto req = std::make_shared<Segment::Request>();
    req->name = seg_name_->text().trimmed().toStdString();
    cli->async_send_request(req, [this, open](rclcpp::Client<Segment>::SharedFuture f) {
        const auto r = f.get();
        report(r->ok, r->ok ? QString("segment %1 %2")
                                  .arg(QString::fromStdString(r->name),
                                       open ? "opened" : "closed")
                            : QString::fromStdString(r->message));
        if (r->ok && !open) seg_name_->clear();
    });
}

PLUGINLIB_EXPORT_CLASS(GnssMissionModule, rover_hmi_core::GuiModule)
