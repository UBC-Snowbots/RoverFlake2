// dwindle_tree.h — generic Hyprland-style dwindle BSP tree over QWidget leaves.
// Extracted from tiling_container.cpp so the panel tiling and the camera grid
// share one mechanism. The tree owns its nodes, never its leaf widgets.
// JSON format matches the original saved-layout format ("panel" leaf key).
#pragma once

#include <QWidget>
#include <QRect>
#include <QPoint>
#include <QJsonObject>
#include <array>
#include <functional>
#include <vector>

// Ops a focused-panel module may consume before the panel tiling acts on them
// (Alt+Arrow chords, Alt+J). dx/dy carry the arrow direction; 0,0 for ToggleSplit.
enum class TilingOp { Focus, Resize, Swap, ToggleSplit };

// One node: internal split (isNode) or leaf wrapping a widget.
// splitTop true = top/bottom split; splitRatio 0.1..1.9, 1.0 = 50/50.
struct DwindleNode {
    DwindleNode* parent = nullptr;
    bool isNode = false;
    std::array<DwindleNode*, 2> children = {nullptr, nullptr};
    bool splitTop = false;
    float splitRatio = 1.0f;
    QRect box;
    QWidget* widget = nullptr;  // leaf nodes only

    DwindleNode* sibling() const {
        if (!parent) return nullptr;
        return parent->children[0] == this ? parent->children[1] : parent->children[0];
    }
};

class DwindleTree {
public:
    static constexpr float RATIO_MIN   = 0.1f;
    static constexpr float RATIO_MAX   = 1.9f;
    static constexpr float RESIZE_STEP = 0.08f;  // per arrow press

    ~DwindleTree() { clear(); }

    // Node factories for callers assembling an initial tree by hand
    // (e.g. the hint-partitioned startup layout). Nodes are tree-owned.
    DwindleNode* makeLeaf(QWidget* w);
    DwindleNode* makeSplit(bool split_top, float ratio, DwindleNode* c0, DwindleNode* c1);
    void setRoot(DwindleNode* n) { root_ = n; }
    DwindleNode* root() const { return root_; }
    bool empty() const { return root_ == nullptr; }

    // Bisect the leaf of split_target (fallback: first leaf); smart split —
    // taller box splits top/bottom, wider splits left/right.
    void add(QWidget* w, QWidget* split_target);
    // Collapse the leaf's parent; sibling takes its box. Returns the sibling
    // subtree's first leaf widget (refocus candidate), or nullptr.
    QWidget* remove(QWidget* w);
    // Collapse every leaf whose widget is null (deserialize leaves them behind
    // when an id no longer resolves — e.g. a renamed camera in a saved layout).
    void pruneNullLeaves();
    void swap(QWidget* a, QWidget* b);
    bool toggleSplit(QWidget* w);
    // Arrow resize: nearest matching-orientation ancestor split; sign flips by
    // which side the leaf is on so it grows toward the arrow.
    bool resizeStep(QWidget* w, int dx, int dy, float step = RESIZE_STEP);
    // Mouse-drag resize: per axis, full-box drag maps to the full ratio range.
    bool resizeDrag(QWidget* w, QPoint delta);

    // Nearest visible leaf widget strictly in the arrow direction; nullptr if none.
    QWidget* nearestInDirection(QWidget* from, int dx, int dy) const;
    DwindleNode* leafFor(QWidget* w) const;
    std::vector<QWidget*> leaves() const;

    // Assign geometry: root takes `area`, leaves get their box inset by `gap`.
    void recalc(QRect area, int gap);
    void setMinPane(int w, int h) { min_pane_w_ = w; min_pane_h_ = h; }

    void clear();

    // Same JSON shape the saved-layout store already contains:
    // {isNode, splitTop, splitRatio, children:[..]} / {isNode:false, panel:<id>}
    QJsonObject serialize(const std::function<QString(QWidget*)>& idFor) const;
    void deserialize(const QJsonObject& obj,
                     const std::function<QWidget*(const QString&)>& widgetFor);

private:
    QWidget* removeLeafNode(DwindleNode* leaf);
    QJsonObject serializeNode(const DwindleNode* n,
                              const std::function<QString(QWidget*)>& idFor) const;
    DwindleNode* deserializeNode(const QJsonObject& obj,
                                 const std::function<QWidget*(const QString&)>& widgetFor);
    void recalcNode(DwindleNode* n, int gap);
    // Nearest ancestor split of the given orientation; child_on_path receives
    // that ancestor's child on the path from the leaf.
    static DwindleNode* splitAncestor(DwindleNode* leaf, bool split_top,
                                      DwindleNode** child_on_path = nullptr);

    DwindleNode* root_ = nullptr;
    std::vector<DwindleNode*> all_nodes_;
    int min_pane_w_ = 80;
    int min_pane_h_ = 60;
};
