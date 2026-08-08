// test_dwindle_tree.cpp — pure-geometry unit tests for the extracted dwindle
// core. Runs headless: offscreen Qt platform, QApplication created lazily
// (gtest_main owns main()).
#include <gtest/gtest.h>

#include <QApplication>
#include <QJsonObject>
#include <QWidget>

#include <rover_hmi_core/dwindle_tree.h>

namespace {

void ensureApp() {
    static int argc = 1;
    static char arg0[] = "test_dwindle_tree";
    static char* argv[] = {arg0, nullptr};
    static QApplication* app = [] {
        qputenv("QT_QPA_PLATFORM", "offscreen");
        return new QApplication(argc, argv);
    }();
    (void)app;
}

class DwindleTreeTest : public ::testing::Test {
protected:
    void SetUp() override {
        ensureApp();
        host = new QWidget();
        host->resize(800, 600);
        host->show();
    }
    void TearDown() override { delete host; }

    QWidget* cell() {
        auto* w = new QWidget(host);
        w->show();
        return w;
    }

    QWidget* host = nullptr;
    DwindleTree tree;
};

TEST_F(DwindleTreeTest, SingleLeafFillsArea) {
    auto* a = cell();
    tree.add(a, nullptr);
    tree.recalc(QRect(0, 0, 800, 600), 0);
    EXPECT_EQ(a->geometry(), QRect(0, 0, 800, 600));
}

TEST_F(DwindleTreeTest, AddSplitsFocusedLeafSmartly) {
    auto* a = cell();
    auto* b = cell();
    tree.add(a, nullptr);
    tree.recalc(QRect(0, 0, 800, 600), 0);
    tree.add(b, a);  // a's box is wider than tall → left/right split
    tree.recalc(QRect(0, 0, 800, 600), 0);
    EXPECT_EQ(a->geometry(), QRect(0, 0, 400, 600));
    EXPECT_EQ(b->geometry(), QRect(400, 0, 400, 600));
}

TEST_F(DwindleTreeTest, RemoveCollapsesSibling) {
    auto* a = cell();
    auto* b = cell();
    tree.add(a, nullptr);
    tree.recalc(QRect(0, 0, 800, 600), 0);
    tree.add(b, a);
    QWidget* next = tree.remove(b);
    EXPECT_EQ(next, a);
    tree.recalc(QRect(0, 0, 800, 600), 0);
    EXPECT_EQ(a->geometry(), QRect(0, 0, 800, 600));
    EXPECT_EQ(tree.leaves().size(), 1u);
}

TEST_F(DwindleTreeTest, RemoveLastLeafEmptiesTree) {
    auto* a = cell();
    tree.add(a, nullptr);
    EXPECT_EQ(tree.remove(a), nullptr);
    EXPECT_TRUE(tree.empty());
}

TEST_F(DwindleTreeTest, SwapExchangesGeometry) {
    auto* a = cell();
    auto* b = cell();
    tree.add(a, nullptr);
    tree.recalc(QRect(0, 0, 800, 600), 0);
    tree.add(b, a);
    tree.recalc(QRect(0, 0, 800, 600), 0);
    QRect ga = a->geometry(), gb = b->geometry();
    tree.swap(a, b);
    tree.recalc(QRect(0, 0, 800, 600), 0);
    EXPECT_EQ(a->geometry(), gb);
    EXPECT_EQ(b->geometry(), ga);
}

TEST_F(DwindleTreeTest, ResizeStepMovesSplitTowardArrow) {
    auto* a = cell();
    auto* b = cell();
    tree.add(a, nullptr);
    tree.recalc(QRect(0, 0, 800, 600), 0);
    tree.add(b, a);  // left/right split at ratio 1.0 → 400/400
    tree.recalc(QRect(0, 0, 800, 600), 0);
    ASSERT_TRUE(tree.resizeStep(a, 1, 0));  // grow a rightward
    tree.recalc(QRect(0, 0, 800, 600), 0);
    EXPECT_GT(a->geometry().width(), 400);
    EXPECT_EQ(a->geometry().width() + b->geometry().width(), 800);
}

TEST_F(DwindleTreeTest, ResizeClampsAtMinPane) {
    auto* a = cell();
    auto* b = cell();
    tree.setMinPane(100, 100);
    tree.add(a, nullptr);
    tree.recalc(QRect(0, 0, 800, 600), 0);
    tree.add(b, a);
    for (int i = 0; i < 100; ++i) tree.resizeStep(a, -1, 0);  // shrink far past clamp
    tree.recalc(QRect(0, 0, 800, 600), 0);
    EXPECT_GE(a->geometry().width(), 40);  // ratio clamp (0.1) → 40px on 800
    EXPECT_GE(b->geometry().width(), 100);
}

TEST_F(DwindleTreeTest, ToggleSplitFlipsOrientation) {
    auto* a = cell();
    auto* b = cell();
    tree.add(a, nullptr);
    tree.recalc(QRect(0, 0, 800, 600), 0);
    tree.add(b, a);  // left/right
    ASSERT_TRUE(tree.toggleSplit(a));
    tree.recalc(QRect(0, 0, 800, 600), 0);
    EXPECT_EQ(a->geometry(), QRect(0, 0, 800, 300));  // now top/bottom
    EXPECT_EQ(b->geometry(), QRect(0, 300, 800, 300));
}

TEST_F(DwindleTreeTest, NearestInDirectionRespectsArrow) {
    auto* a = cell();
    auto* b = cell();
    tree.add(a, nullptr);
    tree.recalc(QRect(0, 0, 800, 600), 0);
    tree.add(b, a);  // a left, b right
    tree.recalc(QRect(0, 0, 800, 600), 0);
    EXPECT_EQ(tree.nearestInDirection(a, 1, 0), b);
    EXPECT_EQ(tree.nearestInDirection(a, -1, 0), nullptr);
    EXPECT_EQ(tree.nearestInDirection(b, -1, 0), a);
}

TEST_F(DwindleTreeTest, SerializeRoundTripPreservesStructure) {
    auto* a = cell();
    auto* b = cell();
    auto* c = cell();
    tree.add(a, nullptr);
    tree.recalc(QRect(0, 0, 800, 600), 0);
    tree.add(b, a);
    tree.recalc(QRect(0, 0, 800, 600), 0);
    tree.add(c, b);
    tree.resizeStep(a, 1, 0);
    tree.recalc(QRect(0, 0, 800, 600), 0);

    auto idFor = [&](QWidget* w) {
        return w == a ? QStringLiteral("a")
             : w == b ? QStringLiteral("b") : QStringLiteral("c");
    };
    QJsonObject json = tree.serialize(idFor);

    QRect ga = a->geometry(), gb = b->geometry(), gc = c->geometry();
    DwindleTree other;
    other.deserialize(json, [&](const QString& id) -> QWidget* {
        return id == "a" ? a : id == "b" ? b : id == "c" ? c : nullptr;
    });
    other.recalc(QRect(0, 0, 800, 600), 0);
    EXPECT_EQ(a->geometry(), ga);
    EXPECT_EQ(b->geometry(), gb);
    EXPECT_EQ(c->geometry(), gc);
    EXPECT_EQ(other.leaves().size(), 3u);
}

TEST_F(DwindleTreeTest, PruneNullLeavesDropsUnresolvedIds) {
    auto* a = cell();
    auto* b = cell();
    tree.add(a, nullptr);
    tree.recalc(QRect(0, 0, 800, 600), 0);
    tree.add(b, a);
    QJsonObject json = tree.serialize([&](QWidget* w) {
        return w == a ? QStringLiteral("a") : QStringLiteral("gone");
    });

    DwindleTree other;
    other.deserialize(json, [&](const QString& id) -> QWidget* {
        return id == "a" ? a : nullptr;  // "gone" no longer resolves
    });
    other.pruneNullLeaves();
    other.recalc(QRect(0, 0, 800, 600), 0);
    ASSERT_EQ(other.leaves().size(), 1u);
    EXPECT_EQ(a->geometry(), QRect(0, 0, 800, 600));
}

}  // namespace
