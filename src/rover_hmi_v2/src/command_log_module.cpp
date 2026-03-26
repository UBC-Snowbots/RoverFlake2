#include "command_log_module.h"
#include "catppuccin.h"

#include <QVBoxLayout>
#include <QFont>
#include <QScrollBar>
#include <QDateTime>

QWidget* CommandLogModule::createWidget(QWidget* parent) {
    auto* widget = new QWidget(parent);
    auto* layout = new QVBoxLayout(widget);
    layout->setContentsMargins(0, 0, 0, 0);

    log_ = new QTextEdit();
    log_->setReadOnly(true);
    log_->setFont(QFont("monospace", theme::FontSizeSm));
    log_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    log_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    log_->setStyleSheet(
        QString("QTextEdit { background: %1; color: %2; border: none; padding: 8px; }"
                "QScrollBar:vertical { background: %1; width: 8px; }"
                "QScrollBar::handle:vertical { background: %3; border-radius: 4px; min-height: 20px; }"
                "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }")
        .arg(theme::BgPanel).arg(theme::TextDim).arg(theme::BorderDim));

    layout->addWidget(log_);
    return widget;
}

void CommandLogModule::setDataBus(MoteusDataBus* bus) {
    QObject::connect(bus, &MoteusDataBus::commandLogged,
                     [this](const QString& cmd) {
        if (!log_) return;
        QString time = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");

        // Color-code based on command type
        QString color = theme::TextDim;
        if (cmd.contains("d stop")) color = theme::Red;
        else if (cmd.contains("d pos")) color = theme::Green;
        else if (cmd.startsWith("#")) color = theme::Cyan;

        log_->append(QString("<span style='color:%1'>%2</span>  <span style='color:%3'>%4</span>")
                     .arg(theme::BorderDim).arg(time).arg(color).arg(cmd.toHtmlEscaped()));

        // Auto-scroll to bottom
        auto* sb = log_->verticalScrollBar();
        sb->setValue(sb->maximum());
    });
}
