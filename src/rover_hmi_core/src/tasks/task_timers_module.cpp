// task_timers_module.cpp — "Task Timers"

#include "task_timers_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QHBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace {

QLabel* sectionLabel(const QString& text) {
    auto* l = new QLabel(text);
    l->setStyleSheet(QString("color: %1; font-size: %2px; font-weight: bold;")
                         .arg(theme::TextDim).arg(theme::FontSizeSm));
    return l;
}

QString mmss(qint64 ms) {
    const qint64 s = (ms + 999) / 1000;
    return QString("%1:%2").arg(s / 60, 2, 10, QChar('0')).arg(s % 60, 2, 10, QChar('0'));
}

}  // namespace

QWidget* TaskTimersModule::createWidget(QWidget* parent) {
    auto* widget = new QWidget(parent);
    widget->setStyleSheet(QString("background: %1;").arg(theme::Bg));
    auto* layout = new QVBoxLayout(widget);
    layout->setSpacing(10);
    layout->setContentsMargins(16, 16, 16, 16);
    clock_.start();

    // --- countdown timers ---
    layout->addWidget(sectionLabel("ORDER TIMERS"));
    auto* add_row = new QHBoxLayout();
    name_in_ = new QLineEdit();
    name_in_->setPlaceholderText("order name");
    name_in_->setStyleSheet(QString("background: %1; color: %2; border: 1px solid %3;"
                                    " border-radius: 6px; padding: 8px;")
                                .arg(theme::BgPanel, theme::Text, theme::BorderDim));
    min_in_ = new QSpinBox();
    min_in_->setRange(0, 99);
    min_in_->setValue(5);
    min_in_->setSuffix("m");
    sec_in_ = new QSpinBox();
    sec_in_->setRange(0, 59);
    sec_in_->setSuffix("s");
    auto* add_btn = new QPushButton("Add");
    QObject::connect(add_btn, &QPushButton::clicked, [this]() {
        const int secs = min_in_->value() * 60 + sec_in_->value();
        if (secs <= 0) return;
        const QString name =
            name_in_->text().trimmed().isEmpty()
                ? QString("order %1").arg(rows_.size() + 1)
                : name_in_->text().trimmed();
        addTimer(name, secs);
        name_in_->clear();
    });
    add_row->addWidget(name_in_, 1);
    add_row->addWidget(min_in_);
    add_row->addWidget(sec_in_);
    add_row->addWidget(add_btn);
    layout->addLayout(add_row);

    rows_layout_ = new QVBoxLayout();
    rows_layout_->setSpacing(6);
    layout->addLayout(rows_layout_);

    // --- credit-tap cadence (one credit per 5 s) ---
    layout->addWidget(sectionLabel("CREDIT TAP CADENCE (1 credit / 5 s)"));
    auto* cad_row = new QHBoxLayout();
    cad_btn_ = new QPushButton("Start");
    QObject::connect(cad_btn_, &QPushButton::clicked, [this]() {
        cadence_on_ = !cadence_on_;
        cadence_start_ = clock_.elapsed();
        cad_btn_->setText(cadence_on_ ? "Stop" : "Start");
        if (!cadence_on_) cad_bar_->setValue(0);
    });
    cad_row->addWidget(cad_btn_);
    cad_bar_ = new QProgressBar();
    cad_bar_->setRange(0, 5000);
    cad_bar_->setTextVisible(false);
    cad_bar_->setStyleSheet(QString(
        "QProgressBar { background: %1; border: 1px solid %2; border-radius: 4px; }"
        "QProgressBar::chunk { background: %3; }")
        .arg(theme::BgPanel, theme::BorderDim, theme::Green));
    cad_row->addWidget(cad_bar_, 1);
    layout->addLayout(cad_row);

    // --- double-tap interval (percussive maintenance, taps ≤ 1 s apart) ---
    layout->addWidget(sectionLabel("DOUBLE-TAP INTERVAL (aim ≤ 1.0 s)"));
    auto* tap_row = new QHBoxLayout();
    auto* tap_btn = new QPushButton("TAP");
    QObject::connect(tap_btn, &QPushButton::clicked, [this]() {
        const qint64 now = clock_.elapsed();
        if (last_tap_ms_ >= 0) {
            const double dt = (now - last_tap_ms_) / 1000.0;
            tap_lbl_->setText(QString("Δ %1 s").arg(dt, 0, 'f', 2));
            tap_lbl_->setStyleSheet(QString("color: %1; font-size: %2px; font-weight: bold;")
                                        .arg(dt <= 1.0 ? theme::Green : theme::Red)
                                        .arg(theme::FontSizeLg));
        }
        last_tap_ms_ = now;
    });
    tap_row->addWidget(tap_btn);
    tap_lbl_ = new QLabel("Δ —");
    tap_lbl_->setStyleSheet(QString("color: %1; font-size: %2px;")
                                .arg(theme::TextDim).arg(theme::FontSizeLg));
    tap_row->addWidget(tap_lbl_, 1);
    layout->addLayout(tap_row);

    layout->addStretch(1);

    tick_timer_ = new QTimer(widget);
    tick_timer_->setInterval(100);
    QObject::connect(tick_timer_, &QTimer::timeout, [this]() { tick(); });
    tick_timer_->start();
    return widget;
}

void TaskTimersModule::addTimer(const QString& name, int secs) {
    auto* row = new Row();
    row->total_ms = row->remaining_ms = qint64(secs) * 1000;

    row->widget = new QWidget();
    auto* h = new QHBoxLayout(row->widget);
    h->setContentsMargins(0, 0, 0, 0);
    h->setSpacing(8);

    auto* name_lbl = new QLabel(name);
    name_lbl->setStyleSheet(QString("color: %1;").arg(theme::Text));
    h->addWidget(name_lbl, 1);

    row->time_lbl = new QLabel(mmss(row->remaining_ms));
    row->time_lbl->setStyleSheet(QString("color: %1; font-size: %2px; font-weight: bold;")
                                     .arg(theme::Green).arg(theme::FontSizeLg));
    h->addWidget(row->time_lbl);

    row->toggle = new QPushButton("▶");
    row->toggle->setFixedWidth(52);
    QObject::connect(row->toggle, &QPushButton::clicked, [this, row]() {
        if (row->running) {
            row->remaining_ms = qMax<qint64>(0, row->end_ms - clock_.elapsed());
        } else {
            row->end_ms = clock_.elapsed() + row->remaining_ms;
        }
        row->running = !row->running;
        row->toggle->setText(row->running ? "⏸" : "▶");
    });
    h->addWidget(row->toggle);

    auto* reset = new QPushButton("↺");
    reset->setFixedWidth(52);
    QObject::connect(reset, &QPushButton::clicked, [this, row]() {
        row->running = false;
        row->remaining_ms = row->total_ms;
        row->toggle->setText("▶");
        updateRow(row);
    });
    h->addWidget(reset);

    auto* remove = new QPushButton("✕");
    remove->setFixedWidth(52);
    QObject::connect(remove, &QPushButton::clicked, [this, row]() {
        rows_.removeOne(row);
        row->widget->deleteLater();
        delete row;
    });
    h->addWidget(remove);

    rows_layout_->addWidget(row->widget);
    rows_.push_back(row);
}

void TaskTimersModule::tick() {
    for (Row* row : rows_) updateRow(row);
    if (cadence_on_) {
        const qint64 ph = (clock_.elapsed() - cadence_start_) % 5000;
        cad_bar_->setValue(int(ph));
    }
}

void TaskTimersModule::updateRow(Row* row) {
    const qint64 rem =
        row->running ? qMax<qint64>(0, row->end_ms - clock_.elapsed()) : row->remaining_ms;
    row->time_lbl->setText(mmss(rem));
    const char* color = theme::Green;
    if (rem == 0)
        color = (clock_.elapsed() / 400) % 2 ? theme::Red : theme::TextDim;  // expired: flash
    else if (rem <= 30000)
        color = theme::Yellow;
    row->time_lbl->setStyleSheet(QString("color: %1; font-size: %2px; font-weight: bold;")
                                     .arg(color).arg(theme::FontSizeLg));
}

PLUGINLIB_EXPORT_CLASS(TaskTimersModule, rover_hmi_core::GuiModule)
