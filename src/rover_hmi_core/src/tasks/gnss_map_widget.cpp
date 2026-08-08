// gnss_map_widget.cpp — offline tile map canvas for the GNSS Mission module

#include "rover_hmi_core/tasks/gnss_map_widget.h"
#include <rover_hmi_core/catppuccin.h>

#include <QDir>
#include <QFile>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>
#include <cmath>

namespace {

constexpr int kTilePx = 256;

// lat/lon -> fractional slippy tile coordinates at zoom z
QPointF llToTile(double lat, double lon, int z) {
    const double n = std::pow(2.0, z);
    const double x = (lon + 180.0) / 360.0 * n;
    const double rad = lat * M_PI / 180.0;
    const double y = (1.0 - std::asinh(std::tan(rad)) / M_PI) / 2.0 * n;
    return {x, y};
}

QPointF tileToLL(double x, double y, int z) {
    const double n = std::pow(2.0, z);
    const double lon = x / n * 360.0 - 180.0;
    const double lat = std::atan(std::sinh(M_PI * (1.0 - 2.0 * y / n)))
                       * 180.0 / M_PI;
    return {lat, lon};
}

QColor categoryColor(const QString& c) {
    if (c == "start")    return QColor(theme::Yellow);
    if (c == "site")     return QColor(theme::Green);
    if (c == "sample")   return QColor(theme::Cyan);
    if (c == "obstacle") return QColor(theme::Red);
    return QColor(theme::Text);  // landmark
}

}  // namespace

GnssMapWidget::GnssMapWidget(QWidget* parent) : QWidget(parent) {
    setMinimumHeight(200);
    setCursor(Qt::OpenHandCursor);
}

// Scan imagery/<site>/tiles: zoom range + center per site (center from the
// max-zoom tile index range). All sites draw together; the "active" one is
// only the badge label and the fetch-tiles target.
void GnssMapWidget::setImageryRoot(const QString& dir) {
    imagery_root_ = dir;
    sites_.clear();
    active_ = -1;
    cache_.clear();
    for (const QString& name : QDir(dir).entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
        const QString tiles = dir + "/" + name + "/tiles";
        QList<int> zooms;
        for (const QString& d : QDir(tiles).entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
            bool num = false;
            const int z = d.toInt(&num);
            if (num) zooms << z;
        }
        if (zooms.isEmpty()) continue;
        std::sort(zooms.begin(), zooms.end());

        const QDir zdir(tiles + "/" + QString::number(zooms.last()));
        double xmin = 1e18, xmax = -1e18, ymin = 1e18, ymax = -1e18;
        for (const QString& xs : zdir.entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
            bool num = false;
            const int x = xs.toInt(&num);
            if (!num) continue;
            xmin = std::min(xmin, double(x)); xmax = std::max(xmax, double(x));
            for (const QString& f : QDir(zdir.filePath(xs)).entryList(QDir::Files)) {
                const int y = f.section('.', 0, 0).toInt(&num);
                if (!num) continue;
                ymin = std::min(ymin, double(y)); ymax = std::max(ymax, double(y));
            }
        }
        if (xmax < xmin) continue;
        const int z = zooms.last();
        const QPointF ll = tileToLL((xmin + xmax + 1) / 2.0,
                                    (ymin + ymax + 1) / 2.0, z);
        const QPointF nw = tileToLL(xmin, ymin, z);          // lat max, lon min
        const QPointF se = tileToLL(xmax + 1, ymax + 1, z);  // lat min, lon max
        sites_ << Site{name, tiles, ll.x(), ll.y(),
                       se.x(), nw.y(), nw.x(), se.y(),
                       zooms.first(), z};
    }
    // Zoom range spans every site so meshed neighbors never clamp each other.
    if (!sites_.isEmpty()) {
        zoom_min_ = sites_[0].zmin;
        zoom_max_ = sites_[0].zmax;
        for (const Site& s : sites_) {
            zoom_min_ = std::min(zoom_min_, s.zmin);
            zoom_max_ = std::max(zoom_max_, s.zmax);
        }
        zoom_ = std::clamp(zoom_, zoom_min_, zoom_max_);
    }
    // No fix yet: open on the first site so there is something to look at.
    if (!have_view_ && !sites_.isEmpty()) {
        center_lat_ = sites_[0].lat;
        center_lon_ = sites_[0].lon;
        have_view_ = true;
    }
    updateActiveSite();
    update();
}

void GnssMapWidget::rescan() {
    if (!imagery_root_.isEmpty()) setImageryRoot(imagery_root_);
}

QString GnssMapWidget::activeSiteName() const {
    return active_ >= 0 ? sites_[active_].name : QString();
}

// Best site for the view center — badge label + fetch-tiles target only,
// tiles composite across all sites regardless. Sites whose tiles COVER the
// point beat merely-nearby ones; nearest center breaks ties and is the
// fallback when nothing covers. Cheap enough to run every repaint.
void GnssMapWidget::updateActiveSite() {
    if (sites_.isEmpty()) return;
    const double cs = std::cos(center_lat_ * M_PI / 180.0);
    int best = -1;
    double best_d = 1e18;
    for (int pass = 0; pass < 2 && best < 0; ++pass) {
        for (int i = 0; i < sites_.size(); ++i) {
            const Site& s = sites_[i];
            if (pass == 0 && (center_lat_ < s.lat0 || center_lat_ > s.lat1 ||
                              center_lon_ < s.lon0 || center_lon_ > s.lon1))
                continue;
            const double dlat = s.lat - center_lat_;
            const double dlon = (s.lon - center_lon_) * cs;
            const double d = dlat * dlat + dlon * dlon;
            if (best < 0 || d < best_d) { best_d = d; best = i; }
        }
    }
    active_ = best;
}

void GnssMapWidget::addFix(double lat, double lon) {
    fix_lat_ = lat; fix_lon_ = lon; have_fix_ = true;
    // ~1e-6 deg ≈ 0.1 m: skip jitter so the path doesn't grow unbounded
    if (path_.isEmpty() ||
        std::abs(path_.last().y() - lat) > 2e-6 ||
        std::abs(path_.last().x() - lon) > 2e-6)
        path_ << QPointF(lon, lat);
    if (follow_) { center_lat_ = lat; center_lon_ = lon; have_view_ = true; }
    update();
}

void GnssMapWidget::addWaypoint(double lat, double lon, const QString& category,
                                const QString& label) {
    waypoints_ << Waypoint{lat, lon, category, label};
    update();
}

void GnssMapWidget::clearRun() {
    path_.clear();
    waypoints_.clear();
    update();
}

void GnssMapWidget::centerOnFix() {
    follow_ = true;
    if (have_fix_) { center_lat_ = fix_lat_; center_lon_ = fix_lon_; }
    update();
}

QPointF GnssMapWidget::toScreen(double lat, double lon) const {
    const QPointF t = llToTile(lat, lon, zoom_);
    const QPointF c = llToTile(center_lat_, center_lon_, zoom_);
    return {width() / 2.0 + (t.x() - c.x()) * kTilePx,
            height() / 2.0 + (t.y() - c.y()) * kTilePx};
}

// A z/x/y tile names one square of earth, so the first site holding it is as
// good as any other — overlapping fetches mesh for free.
const QPixmap* GnssMapWidget::tilePixmap(int z, int x, int y) {
    const QString key = QString("%1/%2/%3").arg(z).arg(x).arg(y);
    auto it = cache_.find(key);
    if (it == cache_.end()) {
        QPixmap pm;
        for (const Site& s : sites_) {
            if (z < s.zmin || z > s.zmax) continue;
            for (const char* ext : {"png", "jpg", "jpeg"}) {
                const QString path = s.tiles + "/" + key + "." + ext;
                if (QFile::exists(path)) { pm.load(path); break; }
            }
            if (!pm.isNull()) break;
        }
        if (cache_.size() > 1024) cache_.clear();
        it = cache_.insert(key, pm);  // null pixmap caches the miss too
    }
    return it->isNull() ? nullptr : &it.value();
}

// Missing at the current zoom: magnify the nearest coarser tile any site
// has, so sites with different zoom ranges (or fetch gaps at their seams)
// blend instead of leaving holes.
bool GnssMapWidget::drawCoarser(QPainter& p, int tx, int ty, const QRectF& dst) {
    for (int dz = 1; dz <= 5 && zoom_ - dz >= zoom_min_; ++dz) {
        if (const QPixmap* pm = tilePixmap(zoom_ - dz, tx >> dz, ty >> dz)) {
            const int f = 1 << dz;
            const double sub = pm->width() / double(f);
            p.drawPixmap(dst, *pm, QRectF((tx & (f - 1)) * sub,
                                          (ty & (f - 1)) * sub, sub, sub));
            return true;
        }
    }
    return false;
}

void GnssMapWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.fillRect(rect(), QColor(theme::BgPanel));

    if (!have_view_) {
        p.setPen(QColor(theme::TextDim));
        p.drawText(rect(), Qt::AlignCenter,
                   "no imagery and no fix yet\n"
                   "(ros2 run rover_mapping fetch_tiles -- --name <site> --center <lat> <lon>)");
        return;
    }
    updateActiveSite();

    // Tiles
    const QPointF c = llToTile(center_lat_, center_lon_, zoom_);
    const int rx = width() / (2 * kTilePx) + 1, ry = height() / (2 * kTilePx) + 1;
    bool drew_tile = false;
    for (int dx = -rx; dx <= rx; ++dx) {
        for (int dy = -ry; dy <= ry; ++dy) {
            const int tx = int(std::floor(c.x())) + dx, ty = int(std::floor(c.y())) + dy;
            const double sx = width() / 2.0 + (tx - c.x()) * kTilePx;
            const double sy = height() / 2.0 + (ty - c.y()) * kTilePx;
            const QRectF dst(sx, sy, kTilePx, kTilePx);
            if (const QPixmap* pm = tilePixmap(zoom_, tx, ty)) {
                p.drawPixmap(dst, *pm, pm->rect());
                drew_tile = true;
            } else if (drawCoarser(p, tx, ty, dst)) {
                drew_tile = true;
            }
        }
    }
    if (!drew_tile) {  // graceful no-imagery mode: faint grid, data still drawn
        p.setPen(QColor(theme::BorderDim));
        const double ox = std::fmod(width() / 2.0 - c.x() * kTilePx, kTilePx);
        const double oy = std::fmod(height() / 2.0 - c.y() * kTilePx, kTilePx);
        for (double x = ox; x < width(); x += kTilePx) p.drawLine(QPointF(x, 0), QPointF(x, height()));
        for (double y = oy; y < height(); y += kTilePx) p.drawLine(QPointF(0, y), QPointF(width(), y));
    }

    // Path
    p.setRenderHint(QPainter::Antialiasing);
    if (path_.size() > 1) {
        QPainterPath pp(toScreen(path_[0].y(), path_[0].x()));
        for (int i = 1; i < path_.size(); ++i)
            pp.lineTo(toScreen(path_[i].y(), path_[i].x()));
        p.setPen(QPen(QColor(theme::Cyan), 2.5));
        p.drawPath(pp);
    }

    // Waypoints
    QFont f("monospace", 9, QFont::Bold);
    p.setFont(f);
    for (const Waypoint& w : waypoints_) {
        const QPointF s = toScreen(w.lat, w.lon);
        const QColor col = categoryColor(w.category);
        p.setPen(QPen(QColor(theme::Bg), 2));
        p.setBrush(col);
        p.drawEllipse(s, 6, 6);
        p.setPen(col);
        p.drawText(s + QPointF(9, 4), w.label);
    }

    // Current fix
    if (have_fix_) {
        const QPointF s = toScreen(fix_lat_, fix_lon_);
        p.setPen(QPen(QColor(theme::Bg), 2));
        p.setBrush(QColor(theme::Green));
        p.drawEllipse(s, 7, 7);
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen(QColor(theme::Green), 1.5));
        p.drawEllipse(s, 12, 12);
    }

    // Site + zoom badge
    p.setPen(QColor(theme::TextDim));
    p.drawText(rect().adjusted(0, 0, -6, -4), Qt::AlignRight | Qt::AlignBottom,
               QString("%1z%2%3")
                   .arg(active_ >= 0 ? sites_[active_].name + " · " : "")
                   .arg(zoom_)
                   .arg(follow_ ? " · follow" : ""));
}

void GnssMapWidget::mousePressEvent(QMouseEvent* e) {
    drag_pos_ = e->pos();
    setCursor(Qt::ClosedHandCursor);
}

void GnssMapWidget::mouseMoveEvent(QMouseEvent* e) {
    const QPoint d = e->pos() - drag_pos_;
    drag_pos_ = e->pos();
    follow_ = false;
    const QPointF c = llToTile(center_lat_, center_lon_, zoom_);
    const QPointF ll = tileToLL(c.x() - d.x() / double(kTilePx),
                                c.y() - d.y() / double(kTilePx), zoom_);
    center_lat_ = ll.x(); center_lon_ = ll.y();
    setCursor(Qt::ClosedHandCursor);
    update();
}

void GnssMapWidget::mouseReleaseEvent(QMouseEvent*) {
    setCursor(Qt::OpenHandCursor);
}

void GnssMapWidget::wheelEvent(QWheelEvent* e) {
    const int nz = zoom_ + (e->angleDelta().y() > 0 ? 1 : -1);
    zoom_ = std::clamp(nz, zoom_min_, zoom_max_);
    update();
}
