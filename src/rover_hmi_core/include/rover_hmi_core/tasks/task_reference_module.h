// task_reference_module.h — "Task Reference"
//
// In-HMI cheat sheets rendered from config/tasks/reference/*.md so nobody
// alt-tabs to a PDF mid-run. Section: Tasks. Pure UI, no ROS.
// defaultVisible: false.

#pragma once

#include <rover_hmi_core/gui_module.h>

#include <QComboBox>
#include <QTextBrowser>

class TaskReferenceModule : public rover_hmi_core::GuiModule {
public:
    std::string name()        const override { return "Task Reference"; }
    std::string sectionName() const override { return "Tasks"; }
    std::string layoutHint()  const override { return "bottom"; }
    bool        defaultVisible() const override { return false; }

    QWidget* createWidget(QWidget* parent) override;

    QJsonObject saveState() const override;
    void        restoreState(const QJsonObject& state) override;

private:
    void reloadList();
    void loadDoc(const QString& filename);

    QComboBox*    combo_   = nullptr;
    QTextBrowser* browser_ = nullptr;
    QString       pending_doc_;  // restoreState before/after list load
};
