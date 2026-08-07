// task_scoring_module.h — "Task Scoring"
//
// Interactive CIRC score tracker: one tab per task from
// config/tasks/scoring.json, flat always-visible action checklist with live
// stage/task/grand totals. Only actions are checkable — stage rows are headers.
// Checks autosave to config/tasks/scoring_state.json (in-repo, independent of
// layouts). Section: Tasks. Pure UI, no ROS. defaultVisible: false.

#pragma once

#include <rover_hmi_core/gui_module.h>

#include <QJsonArray>
#include <QLabel>
#include <QTabWidget>
#include <QTreeWidget>

class TaskScoringModule : public rover_hmi_core::GuiModule {
public:
    std::string name()        const override { return "Task Scoring"; }
    std::string sectionName() const override { return "Tasks"; }
    std::string layoutHint()  const override { return "bottom"; }
    bool        defaultVisible() const override { return false; }

    QWidget* createWidget(QWidget* parent) override;

private:
    void buildTabs();
    void recalc();
    void applyChecked(const QJsonArray& checked);
    QJsonArray collectChecked() const;
    void persist() const;

    QTabWidget*         tabs_      = nullptr;
    QList<QTreeWidget*> trees_;     // one per task tab
    QStringList         names_;     // task names for tab titles
    QLabel*             total_     = nullptr;
    int                 grand_max_ = 0;
    bool                updating_  = false;   // recalc/restore guard vs itemChanged
};
