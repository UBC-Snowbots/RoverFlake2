// gnss_mission_module.cpp — "GNSS Mission"

#include "rover_hmi_core/tasks/gnss_mission_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QDirIterator>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QPushButton>
#include <QRegularExpression>
#include <QTimer>
#include <QUrl>
#include <QVBoxLayout>
#include <cstdlib>

#include <pluginlib/class_list_macros.hpp>

namespace {

// mission_manager rejects tags on a fix older than this (stale_fix_sec).
constexpr double kStaleSec = 2.0;

// src/rover_mapping in the source tree ($ROVERFLAKE_ROOT first, then the
// baked sibling of rover_hmi_core) — imagery, missions, and the rover CLI
// live there, not in the install space.
QString mappingRoot() {
    if (const char* root = std::getenv("ROVERFLAKE_ROOT")) {
        const QString c = QString::fromLocal8Bit(root) + "/src/rover_mapping";
        if (QFileInfo::exists(c)) return c;
    }
#ifdef ROVER_HMI_CORE_SOURCE_DIR
    const QString baked =
        QFileInfo(QStringLiteral(ROVER_HMI_CORE_SOURCE_DIR)).path() + "/rover_mapping";
    if (QFileInfo::exists(baked)) return baked;
#endif
    return {};
}

// Categories accepted by /tag_point (mission_io.CATEGORIES).
const struct { const char* cat; const char* color; } kCats[] = {
    {"start", theme::Yellow}, {"site", theme::Green},
    {"sample", theme::Cyan},  {"obstacle", theme::Red},
    {"landmark", theme::Text}};

// One coordinate field: a bare number, or a "lat, lon" / "lat lon" pair
// pasted whole into the lat box (the lon box is then ignored).
// Returns false with `error` set on anything unparseable.
bool parseCoords(const QString& lat_text, const QString& lon_text,
                 double* lat, double* lon, QString* error) {
    QStringList parts = lat_text.split(QRegularExpression("[,;\\s]+"),
                                       Qt::SkipEmptyParts);
    if (parts.size() < 2) parts << lon_text.trimmed();
    if (parts.size() > 2) {
        *error = QString("expected two numbers, got %1").arg(parts.size());
        return false;
    }
    if (parts.size() != 2 || parts[0].isEmpty() || parts[1].isEmpty()) {
        *error = "enter both a latitude and a longitude";
        return false;
    }
    bool ok_lat = false, ok_lon = false;
    *lat = parts[0].toDouble(&ok_lat);
    *lon = parts[1].toDouble(&ok_lon);
    if (!ok_lat || !ok_lon) {
        *error = QString("not a decimal coordinate: %1, %2").arg(parts[0], parts[1]);
        return false;
    }
    if (*lat < -90.0 || *lat > 90.0) {
        *error = QString("latitude %1 out of range (-90..90)").arg(*lat);
        return false;
    }
    if (*lon < -180.0 || *lon > 180.0) {
        *error = QString("longitude %1 out of range (-180..180)").arg(*lon);
        return false;
    }
    // Matches mission_io.coord_error: 0,0 is a parse slip, never a target.
    if (*lat == 0.0 && *lon == 0.0) {
        *error = "refusing to tag 0, 0 — enter a real coordinate";
        return false;
    }
    return true;
}

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
            if (map_ && msg->status.status >= 0)
                map_->addFix(msg->latitude, msg->longitude);
        });
    // Latched by mission_manager — recovers the active mission after an HMI
    // restart mid-run.
    active_sub_ = node->create_subscription<std_msgs::msg::String>(
        "/mission/active", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::String::SharedPtr msg) {
            if (!mission_label_) return;
            const QString name = QString::fromStdString(msg->data);
            mission_label_->setText(name.isEmpty() ? "—" : name);
            mission_label_->setStyleSheet(QString("color:%1;").arg(
                name.isEmpty() ? theme::TextDim : theme::Green));
        });
    start_cli_     = node->create_client<StartMission>("/mission/start");
    stop_cli_      = node->create_client<Trigger>("/mission/stop");
    tag_cli_       = node->create_client<TagPoint>("/tag_point");
    seg_start_cli_ = node->create_client<Segment>("/segment_start");
    seg_end_cli_   = node->create_client<Segment>("/segment_end");
    export_cli_    = node->create_client<ExportMap>("/mission/export");
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
    for (const auto& c : kCats) {
        auto* b = button(c.cat, c.color);
        const QString cat = c.cat;
        QObject::connect(b, &QPushButton::clicked, [this, cat]() { tag(cat); });
        tag_row->addWidget(b);
    }
    col->addLayout(tag_row);

    // Manual coordinate entry: a point the rover isn't standing on — a target
    // handed over by the judges, something read off another map.
    auto* manual_row = new QHBoxLayout();
    manual_lat_ = new QLineEdit();
    manual_lat_->setPlaceholderText("lat (or \"lat, lon\")");
    manual_lat_->setStyleSheet(lineCss);
    manual_row->addWidget(manual_lat_, 1);
    manual_lon_ = new QLineEdit();
    manual_lon_->setPlaceholderText("lon");
    manual_lon_->setStyleSheet(lineCss);
    manual_row->addWidget(manual_lon_, 1);
    manual_label_ = new QLineEdit();
    manual_label_->setPlaceholderText("label (optional)");
    manual_label_->setStyleSheet(lineCss);
    manual_row->addWidget(manual_label_, 1);
    manual_cat_ = new QComboBox();
    for (const auto& c : kCats) manual_cat_->addItem(c.cat);
    manual_cat_->setCurrentText("landmark");
    manual_cat_->setStyleSheet(QString(
        "QComboBox { color: %1; background: %2; border: 1px solid %3;"
        "  border-radius: 4px; padding: 6px; }"
        "QComboBox QAbstractItemView { color: %1; background: %2;"
        "  selection-background-color: %3; }")
        .arg(theme::Text, theme::BgPanel, theme::BorderDim));
    manual_row->addWidget(manual_cat_);
    auto* add_point = button("Add point", theme::Cyan);
    QObject::connect(add_point, &QPushButton::clicked, [this]() { addManualPoint(); });
    manual_row->addWidget(add_point);
    // Enter in either coordinate field adds the point.
    for (QLineEdit* e : {manual_lat_, manual_lon_, manual_label_})
        QObject::connect(e, &QLineEdit::returnPressed, [this]() { addManualPoint(); });
    col->addLayout(manual_row);

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

    map_ = new GnssMapWidget();
    map_->setImageryRoot(mappingRoot() + "/imagery");
    col->addWidget(map_, 1);

    auto* map_row = new QHBoxLayout();
    map_row->addStretch();
    auto* center = button("Center", theme::Cyan);
    QObject::connect(center, &QPushButton::clicked, [this]() {
        map_->centerOnFix();
        report(bool(fix_), fix_ ? "following the rover"
                                : "no GPS fix yet — nothing to center on");
    });
    map_row->addWidget(center);
    // Manual points outlive clearRun(), so they need their own way out.
    auto* clear_pts = button("Clear points", theme::Yellow);
    QObject::connect(clear_pts, &QPushButton::clicked, [this]() {
        const int n = map_->clearManualPoints();
        report(true, n ? QString("removed %1 manual point%2 from the map"
                                 " (saved ones stay in waypoints.yaml)")
                             .arg(n).arg(n == 1 ? "" : "s")
                       : "no manual points on the map");
    });
    map_row->addWidget(clear_pts);
    auto* exp = button("Export report", theme::Green);
    QObject::connect(exp, &QPushButton::clicked, [this]() { exportReport(); });
    map_row->addWidget(exp);
    auto* open = button("Open report", theme::Text);
    QObject::connect(open, &QPushButton::clicked, [this]() { openReport(); });
    map_row->addWidget(open);
    col->addLayout(map_row);

    status_ = new QLabel();
    status_->setFont(QFont("monospace", theme::FontSize - 2));
    status_->setWordWrap(true);
    col->addWidget(status_);

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
    // mission_label_ follows the latched /mission/active topic.
    start_cli_->async_send_request(req, [this](rclcpp::Client<StartMission>::SharedFuture f) {
        const auto r = f.get();
        report(r->ok, QString::fromStdString(r->message));
        if (r->ok) map_->clearRun();
    });
}

void GnssMissionModule::stopMission() {
    if (!stop_cli_->service_is_ready())
        return report(false, "mission_manager not running");
    stop_cli_->async_send_request(std::make_shared<Trigger::Request>(),
                                  [this](rclcpp::Client<Trigger>::SharedFuture f) {
        const auto r = f.get();
        report(r->success, QString::fromStdString(r->message));
    });
}

void GnssMissionModule::tag(const QString& category) {
    if (!tag_cli_->service_is_ready())
        return report(false, "mission_manager not running");
    auto req = std::make_shared<TagPoint::Request>();
    req->category = category.toStdString();
    req->label = tag_label_->text().trimmed().toStdString();
    tag_cli_->async_send_request(req, [this, category](rclcpp::Client<TagPoint>::SharedFuture f) {
        const auto r = f.get();
        report(r->ok, r->ok ? QString("%1 @ %2, %3")
                                  .arg(QString::fromStdString(r->id))
                                  .arg(r->lat, 0, 'f', 6)
                                  .arg(r->lon, 0, 'f', 6)
                            : QString::fromStdString(r->message));
        if (r->ok) {
            map_->addWaypoint(r->lat, r->lon, category,
                              QString::fromStdString(r->id));
            tag_label_->clear();
        }
    });
}

// Manual coordinates: persisted through /tag_point exactly like a fix tag so
// they reach waypoints.yaml and the exported report. The marker goes on the
// map either way — when mission_manager can't record it (not running, no
// mission started), the point is still plotted and the status line says it
// wasn't saved, so a typed target is never silently swallowed.
void GnssMissionModule::addManualPoint() {
    double lat = 0, lon = 0;
    QString error;
    if (!parseCoords(manual_lat_->text(), manual_lon_->text(), &lat, &lon, &error))
        return report(false, error);

    const QString category = manual_cat_->currentText();
    const QString label    = manual_label_->text().trimmed();
    const QString fallback = label.isEmpty() ? category : label;
    const QString where    = QString("%1, %2").arg(lat, 0, 'f', 6).arg(lon, 0, 'f', 6);

    // Cleared now, not in the response callback — the operator may already be
    // typing the next coordinate by the time mission_manager answers.
    manual_lat_->clear();
    manual_lon_->clear();
    manual_label_->clear();

    const auto plot = [this, lat, lon, category](const QString& marker) {
        map_->addWaypoint(lat, lon, category, marker, /*manual=*/true);
        map_->centerOn(lat, lon);
    };

    if (!tag_cli_->service_is_ready()) {
        plot(fallback);
        return report(false, QString("plotted %1 — mission_manager not running,"
                                     " so it was NOT saved").arg(where));
    }
    auto req = std::make_shared<TagPoint::Request>();
    req->category   = category.toStdString();
    req->label      = label.toStdString();
    req->use_manual = true;
    req->manual_lat = lat;
    req->manual_lon = lon;
    tag_cli_->async_send_request(req, [this, plot, fallback, where](
                                          rclcpp::Client<TagPoint>::SharedFuture f) {
        const auto r = f.get();
        if (r->ok) {
            plot(QString::fromStdString(r->id));
            report(true, QString("%1 @ %2").arg(QString::fromStdString(r->id), where));
        } else {
            plot(fallback);
            report(false, QString("plotted %1 but NOT saved: %2")
                              .arg(where, QString::fromStdString(r->message)));
        }
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

// Rendered by mission_manager's /mission/export (Pillow, tiles + track).
void GnssMissionModule::exportReport() {
    if (!export_cli_->service_is_ready())
        return report(false, "mission_manager not running");
    auto req = std::make_shared<ExportMap::Request>();
    req->mission = "last";
    report(true, "exporting…");
    export_cli_->async_send_request(req, [this](rclcpp::Client<ExportMap>::SharedFuture f) {
        const auto r = f.get();
        report(r->ok, QString::fromStdString(r->message));
        if (r->ok) last_report_ = QString::fromStdString(r->path);
    });
}

// Opens the last-exported report/route_map.png in the system viewer;
// falls back to the newest one on disk (e.g. exported from the CLI).
void GnssMissionModule::openReport() {
    QString target = last_report_;
    if (target.isEmpty()) {
        QDateTime newest_time;
        QDirIterator it(mappingRoot() + "/missions", {"route_map.png"},
                        QDir::Files, QDirIterator::Subdirectories);
        while (it.hasNext()) {
            const QString f = it.next();
            const QDateTime t = it.fileInfo().lastModified();
            if (target.isEmpty() || t > newest_time) { target = f; newest_time = t; }
        }
    }
    if (target.isEmpty())
        return report(false, "no exported report found — Export first");
    QDesktopServices::openUrl(QUrl::fromLocalFile(target));
    report(true, target);
}

PLUGINLIB_EXPORT_CLASS(GnssMissionModule, rover_hmi_core::GuiModule)
