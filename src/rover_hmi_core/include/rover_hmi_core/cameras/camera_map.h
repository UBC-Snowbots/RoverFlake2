// camera_map.h — FNAF-style selector: top-down rover silhouette with one
// clickable dot per camera. No Q_OBJECT; selection via std::function.
#pragma once

#include <QWidget>
#include <functional>
#include <vector>

#include <rover_hmi_core/cameras/camera_config.h>

class CameraMap : public QWidget {
public:
    explicit CameraMap(std::vector<rover_hmi_core::camera_config::Camera> cams,
                       QWidget* parent = nullptr);

    std::function<void(int)> onSelect;

    void setActive(int idx);
    void setAlive(const std::vector<bool>& alive);

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;

private:
    QRect  bodyRect() const;             // silhouette bounds inside the widget
    QPointF dotPos(int idx) const;       // camera dot in widget coords

    std::vector<rover_hmi_core::camera_config::Camera> cams_;
    std::vector<bool> alive_;
    int active_ = -1;
};
