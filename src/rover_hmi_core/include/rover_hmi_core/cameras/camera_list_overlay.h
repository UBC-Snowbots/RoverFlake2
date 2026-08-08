// camera_list_overlay.h — semi-transparent camera list floating bottom-right
// of the single-view viewport. Rows "N label": active highlighted, dead dimmed.
// Keeps itself anchored via an event filter on its parent.
#pragma once

#include <QWidget>
#include <vector>

class CameraListOverlay : public QWidget {
public:
    explicit CameraListOverlay(std::vector<QString> labels, QWidget* parent);

    void setActive(int idx);
    void setAlive(const std::vector<bool>& alive);

protected:
    void paintEvent(QPaintEvent*) override;
    bool eventFilter(QObject* obj, QEvent* ev) override;

private:
    void reposition();
    int rowHeight() const;

    std::vector<QString> labels_;
    std::vector<bool> alive_;
    int active_ = -1;
};
