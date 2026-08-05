// task_reference_module.cpp — "Task Reference"

#include "task_reference_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QDir>
#include <QFile>
#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>
#include <cstdlib>

#include <pluginlib/class_list_macros.hpp>

namespace {

// Same resolution order as spectro_paths: $ROVERFLAKE_ROOT, then the baked
// source dir. Read-only, so no ownership adoption is needed.
QString referenceDir() {
    if (const char* root = std::getenv("ROVERFLAKE_ROOT")) {
        const QString c =
            QString::fromLocal8Bit(root) + "/src/rover_hmi_core/config/tasks/reference";
        if (QDir(c).exists()) return c;
    }
#ifdef ROVER_HMI_CORE_SOURCE_DIR
    const QString baked = QStringLiteral(ROVER_HMI_CORE_SOURCE_DIR "/config/tasks/reference");
    if (QDir(baked).exists()) return baked;
#endif
    return {};
}

QString displayName(QString filename) {
    filename.chop(3);  // ".md"
    filename.replace('_', ' ');
    return filename.toUpper();
}

}  // namespace

QWidget* TaskReferenceModule::createWidget(QWidget* parent) {
    auto* widget = new QWidget(parent);
    widget->setStyleSheet(QString("background: %1;").arg(theme::Bg));
    auto* layout = new QVBoxLayout(widget);
    layout->setSpacing(8);
    layout->setContentsMargins(12, 12, 12, 12);

    auto* top = new QHBoxLayout();
    combo_ = new QComboBox();
    QObject::connect(combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
                     [this](int) { loadDoc(combo_->currentData().toString()); });
    top->addWidget(combo_, 1);
    auto* reload = new QPushButton("↺");
    reload->setFixedWidth(52);
    QObject::connect(reload, &QPushButton::clicked, [this]() {
        pending_doc_ = combo_->currentData().toString();
        reloadList();
    });
    top->addWidget(reload);
    layout->addLayout(top);

    browser_ = new QTextBrowser();
    browser_->setOpenExternalLinks(true);
    browser_->setStyleSheet(QString("QTextBrowser { background: %1; color: %2;"
                                    " border: 1px solid %3; border-radius: 6px;"
                                    " font-size: %4px; padding: 8px; }")
                                .arg(theme::BgPanel, theme::Text, theme::BorderDim)
                                .arg(theme::FontSizeSm));
    layout->addWidget(browser_, 1);

    reloadList();
    return widget;
}

void TaskReferenceModule::reloadList() {
    const QString dir = referenceDir();
    combo_->blockSignals(true);
    combo_->clear();
    if (dir.isEmpty()) {
        combo_->blockSignals(false);
        browser_->setMarkdown("**reference dir not found** — set `ROVERFLAKE_ROOT` or rebuild");
        return;
    }
    for (const QString& f : QDir(dir).entryList({"*.md"}, QDir::Files, QDir::Name))
        combo_->addItem(displayName(f), f);
    if (!pending_doc_.isEmpty()) {
        const int idx = combo_->findData(pending_doc_);
        if (idx >= 0) combo_->setCurrentIndex(idx);
        pending_doc_.clear();
    }
    combo_->blockSignals(false);
    loadDoc(combo_->currentData().toString());
}

void TaskReferenceModule::loadDoc(const QString& filename) {
    if (filename.isEmpty()) return;
    QFile f(referenceDir() + "/" + filename);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        browser_->setMarkdown(QString("**cannot open** `%1`").arg(filename));
        return;
    }
    browser_->setMarkdown(QString::fromUtf8(f.readAll()));
}

QJsonObject TaskReferenceModule::saveState() const {
    if (!combo_ || combo_->currentData().toString().isEmpty()) return {};
    return {{"doc", combo_->currentData().toString()}};
}

void TaskReferenceModule::restoreState(const QJsonObject& state) {
    const QString doc = state.value("doc").toString();
    if (doc.isEmpty()) return;
    if (combo_) {
        const int idx = combo_->findData(doc);
        if (idx >= 0) combo_->setCurrentIndex(idx);
    } else {
        pending_doc_ = doc;
    }
}

PLUGINLIB_EXPORT_CLASS(TaskReferenceModule, rover_hmi_core::GuiModule)
