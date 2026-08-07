// task_scoring_module.cpp — "Task Scoring"

#include "task_scoring_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QFile>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMessageBox>
#include <QPushButton>
#include <QSet>
#include <QTabBar>
#include <QVBoxLayout>
#include <cstdlib>

#include <pluginlib/class_list_macros.hpp>

namespace {

// Same resolution order as the old reference dir: $ROVERFLAKE_ROOT, then the
// baked source dir. Read live so rubric edits need no rebuild.
QString scoringPath() {
    if (const char* root = std::getenv("ROVERFLAKE_ROOT")) {
        const QString c =
            QString::fromLocal8Bit(root) + "/src/rover_hmi_core/config/tasks/scoring.json";
        if (QFile::exists(c)) return c;
    }
#ifdef ROVER_HMI_CORE_SOURCE_DIR
    const QString baked = QStringLiteral(ROVER_HMI_CORE_SOURCE_DIR "/config/tasks/scoring.json");
    if (QFile::exists(baked)) return baked;
#endif
    return {};
}

// Checked state lives next to the rubric so scores survive HMI restarts
// (in-repo, like layouts — never ~/.config).
QString statePath() {
    const QString rubric = scoringPath();
    return rubric.isEmpty() ? QString()
                            : QFileInfo(rubric).path() + "/scoring_state.json";
}

constexpr int RolePts = Qt::UserRole;      // action: pts, stage: max
constexpr int RoleMax = Qt::UserRole + 1;  // tree root: task sheet max

}  // namespace

QWidget* TaskScoringModule::createWidget(QWidget* parent) {
    auto* widget = new QWidget(parent);
    widget->setStyleSheet(QString("background: %1;").arg(theme::Bg));
    auto* layout = new QVBoxLayout(widget);
    layout->setSpacing(8);
    layout->setContentsMargins(12, 12, 12, 12);

    auto* top = new QHBoxLayout();
    total_ = new QLabel();
    total_->setTextFormat(Qt::RichText);
    total_->setStyleSheet("QLabel { font-size: 30px; font-weight: bold; }");
    top->addWidget(total_, 1);
    auto* reset = new QPushButton("Reset");
    QObject::connect(reset, &QPushButton::clicked, [this, widget]() {
        if (QMessageBox::question(widget, "Reset scores",
                                  "Uncheck every action?") != QMessageBox::Yes)
            return;
        applyChecked({});
        persist();
    });
    top->addWidget(reset);
    layout->addLayout(top);

    tabs_ = new QTabWidget();
    tabs_->setStyleSheet(QString(
        "QTabWidget::pane { border: 1px solid %1; border-radius: 6px; background: %2; }"
        "QTabBar::tab { background: %3; color: %4; padding: 8px 14px;"
        "  font-size: 17px; font-weight: bold; border: 1px solid %1; border-bottom: none;"
        "  border-top-left-radius: 6px; border-top-right-radius: 6px; margin-right: 2px; }"
        "QTabBar::tab:selected { background: %2; }")
        .arg(theme::BorderDim, theme::BgPanel, theme::Bg, theme::Text));
    layout->addWidget(tabs_, 1);

    buildTabs();
    QFile sf(statePath());
    if (sf.open(QIODevice::ReadOnly))
        applyChecked(QJsonDocument::fromJson(sf.readAll())["checked"].toArray());
    return widget;
}

void TaskScoringModule::buildTabs() {
    QFile f(scoringPath());
    if (!f.open(QIODevice::ReadOnly)) {
        total_->setText(QString("<span style='color:%1'>scoring.json not found</span>")
                            .arg(theme::Red));
        return;
    }
    const QJsonArray tasks = QJsonDocument::fromJson(f.readAll())["tasks"].toArray();

    const QFont stageFont("monospace", theme::FontSize, QFont::Bold);
    const QFont actionFont("monospace", theme::FontSize - 4);

    updating_ = true;
    grand_max_ = 0;
    for (int t = 0; t < tasks.size(); ++t) {
        const QJsonObject task = tasks[t].toObject();
        const int taskMax = task["max"].toInt();
        grand_max_ += taskMax;
        names_ << task["name"].toString();

        auto* tree = new QTreeWidget();
        tree->setColumnCount(2);
        tree->setHeaderHidden(true);
        tree->setWordWrap(true);
        tree->setRootIsDecorated(false);
        tree->setItemsExpandable(false);
        tree->setIndentation(16);
        tree->header()->setStretchLastSection(false);
        tree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
        tree->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
        tree->setStyleSheet(QString(
            "QTreeWidget { background: %1; color: %2; border: none; font-size: 20px; }"
            "QTreeWidget::item { padding: 5px 4px; }"
            "QTreeWidget::item:selected { background: #1a1a1a; color: %2; }"
            "QTreeWidget::indicator { width: 26px; height: 26px;"
            "  border: 1px solid %3; border-radius: 4px; background: %1; }"
            "QTreeWidget::indicator:checked { background: %4; border-color: %4; }")
            .arg(theme::BgPanel, theme::Text, theme::TextDim, theme::Green));
        tree->invisibleRootItem()->setData(0, RoleMax, taskMax);
        QObject::connect(tree, &QTreeWidget::itemChanged, [this](QTreeWidgetItem*, int col) {
            if (col == 0 && !updating_) { recalc(); persist(); }
        });

        for (const QJsonValue& sv : task["stages"].toArray()) {
            const QJsonObject stage = sv.toObject();
            int stageMax = 0;
            if (stage.contains("max")) {
                stageMax = stage["max"].toInt();
            } else {
                for (const QJsonValue& av : stage["actions"].toArray())
                    stageMax += qMax(0, av.toObject()["pts"].toInt());
            }

            auto* stageItem = new QTreeWidgetItem(tree);
            stageItem->setText(0, stage["name"].toString());
            stageItem->setFont(0, stageFont);
            stageItem->setFont(1, stageFont);
            stageItem->setData(0, RolePts, stageMax);
            stageItem->setFlags(stageItem->flags() & ~Qt::ItemIsSelectable);

            for (const QJsonValue& av : stage["actions"].toArray()) {
                const QJsonObject action = av.toObject();
                auto* item = new QTreeWidgetItem(stageItem);
                item->setText(0, action["text"].toString());
                item->setFont(0, actionFont);
                item->setFont(1, actionFont);
                item->setData(0, RolePts, action["pts"].toInt());
                item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
                item->setCheckState(0, Qt::Unchecked);
            }
        }
        tree->expandAll();
        trees_ << tree;
        tabs_->addTab(tree, "");
        tabs_->tabBar()->setTabTextColor(t, QColor(theme::MotorColors[t % 6]));
    }
    updating_ = false;
    recalc();
}

void TaskScoringModule::recalc() {
    updating_ = true;
    int grand = 0;
    for (int t = 0; t < trees_.size(); ++t) {
        QTreeWidget* tree = trees_[t];
        int taskEarned = 0;
        for (int s = 0; s < tree->topLevelItemCount(); ++s) {
            QTreeWidgetItem* stageItem = tree->topLevelItem(s);
            int stageEarned = 0;
            for (int a = 0; a < stageItem->childCount(); ++a) {
                QTreeWidgetItem* item = stageItem->child(a);
                const int pts = item->data(0, RolePts).toInt();
                const bool on = item->checkState(0) == Qt::Checked;
                if (on) stageEarned += pts;
                item->setText(1, QString("%1%2").arg(pts > 0 ? "+" : "").arg(pts));
                item->setForeground(1, QColor(on ? (pts < 0 ? theme::Red : theme::Green)
                                                 : (pts < 0 ? theme::Red : theme::TextDim)));
                item->setForeground(0, QColor(on ? theme::TextDim : theme::Text));
                QFont fnt = item->font(0);
                fnt.setStrikeOut(on);
                item->setFont(0, fnt);
            }
            const int stageMax = stageItem->data(0, RolePts).toInt();
            stageItem->setText(1, QString("%1 / %2").arg(stageEarned).arg(stageMax));
            stageItem->setForeground(1, QColor(stageEarned >= stageMax ? theme::Green
                                               : stageEarned > 0       ? theme::Yellow
                                                                       : theme::TextDim));
            stageItem->setForeground(0, QColor(theme::Cyan));
            taskEarned += stageEarned;
        }
        const int taskMax = tree->invisibleRootItem()->data(0, RoleMax).toInt();
        tabs_->setTabText(t, QString("%1 · %2/%3").arg(names_[t]).arg(taskEarned).arg(taskMax));
        grand += taskEarned;
    }
    total_->setText(QString("<span style='color:%1'>TOTAL </span>"
                            "<span style='color:%2'>%3</span>"
                            "<span style='color:%1'> / %4</span>")
                        .arg(theme::TextDim, theme::Green)
                        .arg(grand)
                        .arg(grand_max_));
    updating_ = false;
}

// Checked actions are stored as "task.stage.action" index strings.
void TaskScoringModule::applyChecked(const QJsonArray& checked) {
    QSet<QString> keys;
    for (const QJsonValue& v : checked) keys.insert(v.toString());
    updating_ = true;
    for (int t = 0; t < trees_.size(); ++t) {
        for (int s = 0; s < trees_[t]->topLevelItemCount(); ++s) {
            QTreeWidgetItem* stageItem = trees_[t]->topLevelItem(s);
            for (int a = 0; a < stageItem->childCount(); ++a)
                stageItem->child(a)->setCheckState(
                    0, keys.contains(QString("%1.%2.%3").arg(t).arg(s).arg(a))
                           ? Qt::Checked : Qt::Unchecked);
        }
    }
    updating_ = false;
    recalc();
}

QJsonArray TaskScoringModule::collectChecked() const {
    QJsonArray checked;
    for (int t = 0; t < trees_.size(); ++t) {
        for (int s = 0; s < trees_[t]->topLevelItemCount(); ++s) {
            QTreeWidgetItem* stageItem = trees_[t]->topLevelItem(s);
            for (int a = 0; a < stageItem->childCount(); ++a)
                if (stageItem->child(a)->checkState(0) == Qt::Checked)
                    checked.append(QString("%1.%2.%3").arg(t).arg(s).arg(a));
        }
    }
    return checked;
}

void TaskScoringModule::persist() const {
    const QString p = statePath();
    if (p.isEmpty()) return;
    QFile f(p);
    if (!f.open(QIODevice::WriteOnly)) return;
    f.write(QJsonDocument(QJsonObject{{"checked", collectChecked()}})
                .toJson(QJsonDocument::Indented));
}

PLUGINLIB_EXPORT_CLASS(TaskScoringModule, rover_hmi_core::GuiModule)
