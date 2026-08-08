// gnss_map_widget.h — offline slippy-tile map canvas for the GNSS Mission
// module: XYZ imagery, the live /gnss_fix path, tagged waypoints, and
// hand-entered points. Scans imagery/ and composites tiles from every site —
// z/x/y is a global grid, so overlapping sites mesh; missing zooms fall back
// to a magnified coarser tile. Drag pans, wheel zooms; following the fix
// resumes via centerOnFix(). Pure QWidget, no ROS — the module feeds it data.

#pragma once

#include <QMap>
#include <QPixmap>
#include <QWidget>

class GnssMapWidget : public QWidget {
public:
    explicit GnssMapWidget(QWidget* parent = nullptr);

    void setImageryRoot(const QString& dir);  // scans imagery/<site>/tiles
    void rescan();                          // pick up freshly fetched tiles
    void addFix(double lat, double lon);
    // manual = entered by hand rather than tagged at the fix; drawn as a
    // diamond so a planned target never reads as somewhere the rover has been.
    void addWaypoint(double lat, double lon, const QString& category,
                     const QString& label, bool manual = false);
    // New mission: drop the path and tagged waypoints. Manual points survive —
    // they are targets for the run about to start, not leftovers from the last.
    void clearRun();
    int  clearManualPoints();               // returns how many were removed
    void centerOnFix();
    void centerOn(double lat, double lon);  // jump to a coordinate, stop following
    bool    haveView() const { return have_view_; }
    double  viewLat()  const { return center_lat_; }
    double  viewLon()  const { return center_lon_; }
    QString activeSiteName() const;         // site under the view, or ""

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void wheelEvent(QWheelEvent*) override;

private:
    struct Waypoint { double lat, lon; QString category, label; bool manual; };
    struct Site {
        QString name, tiles;
        double lat, lon;                    // center
        double lat0, lon0, lat1, lon1;      // coverage bounds (min, max)
        int zmin, zmax;
    };

    QPointF toScreen(double lat, double lon) const;
    const QPixmap* tilePixmap(int z, int x, int y);   // any site, cached
    bool drawCoarser(QPainter& p, int tx, int ty, const QRectF& dst);
    void updateActiveSite();                // badge + fetch target only

    QList<Site> sites_;
    int active_ = -1;
    QString imagery_root_;
    int zoom_min_ = 3, zoom_max_ = 19;
    int zoom_ = 18;
    double center_lat_ = 0, center_lon_ = 0;
    bool have_view_ = false;
    bool follow_ = true;

    QVector<QPointF> path_;                 // x=lon, y=lat
    double fix_lat_ = 0, fix_lon_ = 0;
    bool have_fix_ = false;
    QList<Waypoint> waypoints_;
    QMap<QString, QPixmap> cache_;          // "z/x/y" -> pixmap (null = miss)
    QPoint drag_pos_;
};
