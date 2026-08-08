// camera_picker_overlay.h — grid-view camera picker (toggled with P).
// Centered modal list in the layout-manager idiom: rows "N label" with an
// in-grid checkmark, dead cameras dimmed. Up/Down navigate, Enter/Space
// toggle membership, Esc/P close. Toggling calls back into CameraModule.
#pragma once

#include <QWidget>
#include <QStringList>
#include <functional>
#include <vector>

class CameraPickerOverlay : public QWidget {
public:
    CameraPickerOverlay(QStringList labels, QWidget* parent);

    std::function<void(int)> onToggle;  // set by CameraModule

    void setState(const std::vector<bool>& in_grid, const std::vector<bool>& alive);
    void toggle();  // show + focus, or hide

protected:
    void paintEvent(QPaintEvent*) override;
    void keyPressEvent(QKeyEvent*) override;

private:
    QStringList labels_;
    std::vector<bool> in_grid_;
    std::vector<bool> alive_;
    int focused_idx_ = 0;
};
