// dwindle_tree.cpp — generic dwindle BSP tree; logic extracted verbatim from
// tiling_container.cpp with TilePanel* generalized to QWidget*.
#include <rover_hmi_core/dwindle_tree.h>

#include <QJsonArray>
#include <algorithm>

DwindleNode* DwindleTree::makeLeaf(QWidget* w) {
    auto* leaf = new DwindleNode();
    leaf->widget = w;
    all_nodes_.push_back(leaf);
    return leaf;
}

DwindleNode* DwindleTree::makeSplit(bool split_top, float ratio,
                                    DwindleNode* c0, DwindleNode* c1) {
    auto* node = new DwindleNode();
    node->isNode = true;
    node->splitTop = split_top;
    node->splitRatio = ratio;
    node->children[0] = c0;
    node->children[1] = c1;
    if (c0) c0->parent = node;
    if (c1) c1->parent = node;
    all_nodes_.push_back(node);
    return node;
}

DwindleNode* DwindleTree::leafFor(QWidget* w) const {
    if (!root_ || !w) return nullptr;
    for (auto* n : all_nodes_)
        if (!n->isNode && n->widget == w) return n;
    return nullptr;
}

std::vector<QWidget*> DwindleTree::leaves() const {
    std::vector<QWidget*> out;
    for (auto* n : all_nodes_)
        if (!n->isNode && n->widget) out.push_back(n->widget);
    return out;
}

void DwindleTree::add(QWidget* w, QWidget* split_target) {
    auto* newLeaf = makeLeaf(w);

    if (!root_) {
        root_ = newLeaf;
        return;
    }

    DwindleNode* target = split_target ? leafFor(split_target) : nullptr;
    if (!target) {
        target = root_;
        while (target->isNode && target->children[0]) target = target->children[0];
    }

    // Replace target with an internal node holding {target, newLeaf}.
    auto* newParent = new DwindleNode();
    newParent->isNode = true;
    newParent->parent = target->parent;
    // Smart split: taller box → top/bottom, wider box → left/right
    newParent->splitTop = (target->box.height() >= target->box.width());

    if (target->parent) {
        if (target->parent->children[0] == target)
            target->parent->children[0] = newParent;
        else
            target->parent->children[1] = newParent;
    } else {
        root_ = newParent;
    }

    target->parent  = newParent;
    newLeaf->parent = newParent;
    newParent->children[0] = target;
    newParent->children[1] = newLeaf;
    all_nodes_.push_back(newParent);
}

QWidget* DwindleTree::remove(QWidget* w) {
    auto* leaf = leafFor(w);
    if (!leaf) return nullptr;
    return removeLeafNode(leaf);
}

QWidget* DwindleTree::removeLeafNode(DwindleNode* leaf) {
    if (!leaf->parent) {
        all_nodes_.erase(std::remove(all_nodes_.begin(), all_nodes_.end(), leaf),
                         all_nodes_.end());
        delete leaf;
        root_ = nullptr;
        return nullptr;
    }

    auto* parent  = leaf->parent;
    auto* sibling = leaf->sibling();

    // Sibling takes parent's place
    sibling->parent = parent->parent;
    if (parent->parent) {
        if (parent->parent->children[0] == parent)
            parent->parent->children[0] = sibling;
        else
            parent->parent->children[1] = sibling;
    } else {
        root_ = sibling;
    }

    all_nodes_.erase(std::remove(all_nodes_.begin(), all_nodes_.end(), leaf),
                     all_nodes_.end());
    all_nodes_.erase(std::remove(all_nodes_.begin(), all_nodes_.end(), parent),
                     all_nodes_.end());
    delete leaf;
    delete parent;

    DwindleNode* next = sibling;
    while (next && next->isNode) next = next->children[0];
    return next ? next->widget : nullptr;
}

void DwindleTree::pruneNullLeaves() {
    for (bool changed = true; changed && root_; ) {
        changed = false;
        for (auto* n : all_nodes_) {
            if (!n->isNode && !n->widget) {
                removeLeafNode(n);
                changed = true;
                break;  // all_nodes_ was mutated — restart the scan
            }
        }
    }
}

void DwindleTree::swap(QWidget* a, QWidget* b) {
    auto* nodeA = leafFor(a);
    auto* nodeB = leafFor(b);
    if (!nodeA || !nodeB) return;
    std::swap(nodeA->widget, nodeB->widget);
}

bool DwindleTree::toggleSplit(QWidget* w) {
    auto* leaf = leafFor(w);
    if (!leaf || !leaf->parent) return false;
    leaf->parent->splitTop = !leaf->parent->splitTop;
    return true;
}

DwindleNode* DwindleTree::splitAncestor(DwindleNode* leaf, bool split_top,
                                        DwindleNode** child_on_path) {
    for (auto* cur = leaf; cur && cur->parent; cur = cur->parent) {
        if (cur->parent->splitTop == split_top) {
            if (child_on_path) *child_on_path = cur;
            return cur->parent;
        }
    }
    return nullptr;
}

bool DwindleTree::resizeStep(QWidget* w, int dx, int dy, float step) {
    auto* leaf = leafFor(w);
    if (!leaf) return false;

    int arrow = (dx != 0) ? dx : dy;
    bool split_top = (dx == 0);  // vertical arrow → top/bottom split

    DwindleNode* child = nullptr;
    auto* split = splitAncestor(leaf, split_top, &child);
    if (!split) return false;

    float delta = (split->children[0] == child ? arrow : -arrow) * step;
    split->splitRatio = std::clamp(split->splitRatio + delta, RATIO_MIN, RATIO_MAX);
    return true;
}

bool DwindleTree::resizeDrag(QWidget* w, QPoint delta) {
    auto* leaf = leafFor(w);
    if (!leaf) return false;
    bool changed = false;
    // *2 maps a full-box drag to the full 0..2 ratio range
    if (delta.x() != 0) {
        if (auto* split = splitAncestor(leaf, /*split_top=*/false)) {
            float rel = (float)delta.x() * 2.0f / std::max(1, split->box.width());
            split->splitRatio = std::clamp(split->splitRatio + rel, RATIO_MIN, RATIO_MAX);
            changed = true;
        }
    }
    if (delta.y() != 0) {
        if (auto* split = splitAncestor(leaf, /*split_top=*/true)) {
            float rel = (float)delta.y() * 2.0f / std::max(1, split->box.height());
            split->splitRatio = std::clamp(split->splitRatio + rel, RATIO_MIN, RATIO_MAX);
            changed = true;
        }
    }
    return changed;
}

QWidget* DwindleTree::nearestInDirection(QWidget* from, int dx, int dy) const {
    if (!from) return nullptr;
    QPoint cur = from->geometry().center();

    QWidget* best = nullptr;
    double best_dist = 1e18;
    for (auto* n : all_nodes_) {
        if (n->isNode || !n->widget) continue;
        auto* w = n->widget;
        if (w == from || !w->isVisible()) continue;
        QPoint rel = w->geometry().center() - cur;
        if (dx > 0 && rel.x() <= 0) continue;
        if (dx < 0 && rel.x() >= 0) continue;
        if (dy > 0 && rel.y() <= 0) continue;
        if (dy < 0 && rel.y() >= 0) continue;

        double dist = (double)rel.x() * rel.x() + (double)rel.y() * rel.y();
        if (dist < best_dist) { best_dist = dist; best = w; }
    }
    return best;
}

void DwindleTree::recalcNode(DwindleNode* n, int gap) {
    if (!n->isNode) {
        if (n->widget)
            n->widget->setGeometry(n->box.adjusted(gap, gap, -gap, -gap));
        return;
    }

    auto* c0 = n->children[0];
    auto* c1 = n->children[1];
    if (!c0 || !c1) return;

    if (!n->splitTop) {
        int w0 = (int)(n->box.width() / 2.0f * n->splitRatio);
        // max(min()) not std::clamp: bounds may cross when box < 2*min_pane
        w0 = std::max(min_pane_w_, std::min(w0, n->box.width() - min_pane_w_));
        c0->box = QRect(n->box.x(), n->box.y(), w0, n->box.height());
        c1->box = QRect(n->box.x() + w0, n->box.y(), n->box.width() - w0, n->box.height());
    } else {
        int h0 = (int)(n->box.height() / 2.0f * n->splitRatio);
        h0 = std::max(min_pane_h_, std::min(h0, n->box.height() - min_pane_h_));
        c0->box = QRect(n->box.x(), n->box.y(), n->box.width(), h0);
        c1->box = QRect(n->box.x(), n->box.y() + h0, n->box.width(), n->box.height() - h0);
    }

    recalcNode(c0, gap);
    recalcNode(c1, gap);
}

void DwindleTree::recalc(QRect area, int gap) {
    if (!root_ || area.isEmpty()) return;
    root_->box = area;
    recalcNode(root_, gap);
}

void DwindleTree::clear() {
    for (auto* n : all_nodes_) delete n;
    all_nodes_.clear();
    root_ = nullptr;
}

QJsonObject DwindleTree::serializeNode(
        const DwindleNode* node,
        const std::function<QString(QWidget*)>& idFor) const {
    QJsonObject obj;
    obj["isNode"] = node->isNode;
    if (node->isNode) {
        obj["splitTop"]   = node->splitTop;
        obj["splitRatio"] = (double)node->splitRatio;
        QJsonArray children;
        if (node->children[0]) children.append(serializeNode(node->children[0], idFor));
        if (node->children[1]) children.append(serializeNode(node->children[1], idFor));
        obj["children"] = children;
    } else {
        obj["panel"] = node->widget ? idFor(node->widget) : QString();
    }
    return obj;
}

QJsonObject DwindleTree::serialize(const std::function<QString(QWidget*)>& idFor) const {
    return root_ ? serializeNode(root_, idFor) : QJsonObject();
}

DwindleNode* DwindleTree::deserializeNode(
        const QJsonObject& obj,
        const std::function<QWidget*(const QString&)>& widgetFor) {
    auto* node = new DwindleNode();
    all_nodes_.push_back(node);
    node->isNode = obj["isNode"].toBool();
    if (node->isNode) {
        node->splitTop   = obj["splitTop"].toBool();
        node->splitRatio = (float)obj["splitRatio"].toDouble(1.0);
        auto children    = obj["children"].toArray();
        if (children.size() >= 2) {
            node->children[0] = deserializeNode(children[0].toObject(), widgetFor);
            node->children[1] = deserializeNode(children[1].toObject(), widgetFor);
            node->children[0]->parent = node;
            node->children[1]->parent = node;
        }
    } else {
        node->widget = widgetFor(obj["panel"].toString());
    }
    return node;
}

void DwindleTree::deserialize(
        const QJsonObject& obj,
        const std::function<QWidget*(const QString&)>& widgetFor) {
    clear();
    if (!obj.isEmpty()) root_ = deserializeNode(obj, widgetFor);
}
