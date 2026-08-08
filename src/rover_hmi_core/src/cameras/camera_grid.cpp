#include "camera_grid.h"
#include "camera_viewport.h"

#include <QEvent>
#include <QResizeEvent>
#include <algorithm>

namespace {
constexpr int CELL_GAP = 2;
// Camera cells can pack tighter than dashboard panels before becoming useless.
constexpr int MIN_CELL_W = 120;
constexpr int MIN_CELL_H = 90;
}

CameraGrid::CameraGrid(QWidget* parent) : QWidget(parent) {
    tree_.setMinPane(MIN_CELL_W, MIN_CELL_H);
}

void CameraGrid::setCells(const std::vector<CameraViewport*>& cells,
                          const QStringList& names) {
    cells_ = cells;
    names_ = names;
    for (auto* cell : cells_) {
        cell->setParent(this);
        cell->installEventFilter(this);  // hover → focus, like panel tiles
    }
}

int CameraGrid::indexOf(QWidget* w) const {
    for (int i = 0; i < int(cells_.size()); ++i)
        if (cells_[size_t(i)] == w) return i;
    return -1;
}

void CameraGrid::setFocusedCell(CameraViewport* cell) {
    if (focused_ == cell) return;
    focused_ = cell;
    for (auto* c : cells_) c->setFocused(c == focused_);
}

void CameraGrid::setMembership(const std::vector<bool>& in_grid) {
    for (int i = 0; i < int(cells_.size()) && i < int(in_grid.size()); ++i) {
        auto* cell = cells_[size_t(i)];
        const bool want = in_grid[size_t(i)];
        const bool have = tree_.leafFor(cell) != nullptr;
        if (want && !have) {
            tree_.add(cell, focused_);
            cell->setVisible(true);
            setFocusedCell(cell);
        } else if (!want && have) {
            QWidget* next = tree_.remove(cell);
            cell->setVisible(false);
            if (focused_ == cell)
                setFocusedCell(next ? static_cast<CameraViewport*>(next) : nullptr);
        }
    }
    relayout();
}

bool CameraGrid::handleOp(TilingOp op, int dx, int dy) {
    if (!focused_) {
        // No focus yet (e.g. straight after a layout load) — adopt first leaf.
        auto ls = tree_.leaves();
        if (ls.empty()) return true;
        setFocusedCell(static_cast<CameraViewport*>(ls.front()));
    }
    switch (op) {
    case TilingOp::Focus:
        if (auto* w = tree_.nearestInDirection(focused_, dx, dy))
            setFocusedCell(static_cast<CameraViewport*>(w));
        break;
    case TilingOp::Resize:
        if (tree_.resizeStep(focused_, dx, dy)) relayout();
        break;
    case TilingOp::Swap:
        if (auto* w = tree_.nearestInDirection(focused_, dx, dy)) {
            tree_.swap(focused_, w);
            relayout();
        }
        break;
    case TilingOp::ToggleSplit:
        if (tree_.toggleSplit(focused_)) relayout();
        break;
    }
    return true;  // grid view owns the chords, even at an edge
}

void CameraGrid::relayout() {
    tree_.recalc(rect(), CELL_GAP);
}

void CameraGrid::resizeEvent(QResizeEvent*) {
    relayout();
}

bool CameraGrid::eventFilter(QObject* obj, QEvent* ev) {
    if (ev->type() == QEvent::Enter) {
        int idx = indexOf(qobject_cast<QWidget*>(obj));
        if (idx >= 0) setFocusedCell(cells_[size_t(idx)]);
    }
    return false;
}

QJsonObject CameraGrid::saveTree() const {
    return tree_.serialize([this](QWidget* w) {
        int idx = indexOf(w);
        return idx >= 0 ? names_.value(idx) : QString();
    });
}

void CameraGrid::restoreTree(const QJsonObject& obj) {
    // Current members before the tree is replaced; restore keeps this set.
    std::vector<QWidget*> members = tree_.leaves();

    auto isMember = [&members](QWidget* w) {
        return std::find(members.begin(), members.end(), w) != members.end();
    };

    tree_.deserialize(obj, [this, &isMember](const QString& name) -> QWidget* {
        int idx = int(names_.indexOf(name));
        if (idx < 0) return nullptr;
        auto* cell = cells_[size_t(idx)];
        return isMember(cell) ? cell : nullptr;
    });
    tree_.pruneNullLeaves();  // names that resolved to nothing collapse away

    // Append members the saved tree didn't mention.
    for (auto* w : members)
        if (!tree_.leafFor(w)) tree_.add(w, nullptr);

    setFocusedCell(nullptr);
    auto ls = tree_.leaves();
    if (!ls.empty())
        setFocusedCell(static_cast<CameraViewport*>(ls.front()));
    relayout();
}
