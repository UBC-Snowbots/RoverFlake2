// camera_grid.h — dwindle-tiled grid of camera cells. Same DwindleTree the
// panel tiling uses; CameraModule forwards Alt-chord tiling ops here while
// the module is visible. Hover focuses a cell (like panels).
#pragma once

#include <QWidget>
#include <QJsonObject>
#include <QStringList>
#include <vector>

#include <rover_hmi_core/dwindle_tree.h>

class CameraViewport;

class CameraGrid : public QWidget {
public:
    explicit CameraGrid(QWidget* parent = nullptr);

    // All cells + their stable ids (camera names, used in saved layouts).
    // Call once; cells are reparented into this widget.
    void setCells(const std::vector<CameraViewport*>& cells, const QStringList& names);

    // Sync the tree to the wanted membership: absent members are added by
    // splitting the focused cell, removed ones collapse away. Others keep
    // their place — toggling one camera never rebuilds the whole grid.
    void setMembership(const std::vector<bool>& in_grid);

    // Alt-chord ops routed from TilingContainer via CameraModule. Always
    // consumes: while visible the camera panel owns the chords.
    bool handleOp(TilingOp op, int dx, int dy);

    // Hover-focused cell (green border); nullptr when the grid is empty.
    CameraViewport* focusedCell() const { return focused_; }

    // Tree persistence inside the module's saved state.
    QJsonObject saveTree() const;
    // Rebuild from saved JSON: leaves resolve by camera name; names that are
    // not current members are dropped, members missing from the JSON are
    // appended via a normal dwindle add.
    void restoreTree(const QJsonObject& obj);

protected:
    void resizeEvent(QResizeEvent*) override;
    bool eventFilter(QObject* obj, QEvent* ev) override;

private:
    void setFocusedCell(CameraViewport* cell);
    void relayout();
    int indexOf(QWidget* w) const;

    DwindleTree tree_;
    std::vector<CameraViewport*> cells_;
    QStringList names_;
    CameraViewport* focused_ = nullptr;
};
