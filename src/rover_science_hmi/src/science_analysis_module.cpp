#include "science_analysis_module.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QScrollArea>
#include <QFrame>
#include <QSizePolicy>
#include <QFont>
#include <QPainter>
#include <QPaintEvent>
#include <QDateTime>
#include <QDir>
#include <QStandardPaths>
#include <QIODevice>
#include <cmath>
#include <numeric>
#include <limits>

#include <pluginlib/class_list_macros.hpp>
#include <rover_hmi_core/catppuccin.h>

// ─────────────────────────────────────────────────────────────────────────────
// SpectroPaint — bar chart widget (Q_OBJECT in .cpp for CMAKE_AUTOMOC)
// ─────────────────────────────────────────────────────────────────────────────

class SpectroPaint : public QWidget
{
    Q_OBJECT
public:
    explicit SpectroPaint(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        values_.fill(std::numeric_limits<float>::quiet_NaN());
        setMinimumHeight(120);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        setStyleSheet("background: #000000;");
    }

    void setValues(const std::array<float,6>& vals)
    {
        values_ = vals;
        update();
    }

    void clearAll()
    {
        values_.fill(std::numeric_limits<float>::quiet_NaN());
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);

        int W = width();
        int H = height();

        // Background
        p.fillRect(0, 0, W, H, QColor("#000000"));

        const int margin_left  = 40;
        const int margin_right = 10;
        const int margin_top   = 20;
        const int margin_bot   = 30;

        int plot_w = W - margin_left - margin_right;
        int plot_h = H - margin_top  - margin_bot;

        // Grid lines at 0.5 / 1.0 / 1.5 / 2.0 AU
        QPen grid_pen(QColor("#222222"), 1, Qt::DashLine);
        p.setPen(grid_pen);
        for (int tick = 1; tick <= 4; tick++) {
            float y_frac = tick * 0.5f / 2.0f;
            int y = margin_top + static_cast<int>(plot_h * (1.0f - y_frac));
            p.drawLine(margin_left, y, margin_left + plot_w, y);

            // Y axis label
            p.setPen(QColor("#555555"));
            p.setFont(QFont("monospace", 9));
            p.drawText(0, y + 4, margin_left - 4,
                       10, Qt::AlignRight,
                       QString::number(tick * 0.5, 'f', 1));
            p.setPen(grid_pen);
        }

        // Y axis label "AU"
        p.save();
        p.translate(10, margin_top + plot_h / 2);
        p.rotate(-90);
        p.setPen(QColor("#555555"));
        p.setFont(QFont("monospace", 10));
        p.drawText(-15, -2, 30, 14, Qt::AlignCenter, "AU");
        p.restore();

        // 6 bars
        int bar_area_w = plot_w / 6;
        int bar_pad = qMax(2, bar_area_w / 6);

        const char* colors[] = {
            "#5599ff", "#cc77ff", "#00ff88",
            "#ff9944", "#ff4466", "#44ddcc"
        };

        for (int i = 0; i < 6; i++) {
            int x0 = margin_left + i * bar_area_w;
            int bar_x = x0 + bar_pad;
            int bar_w = bar_area_w - 2 * bar_pad;

            float val = values_[i];
            bool has_val = !std::isnan(val);

            // X axis label
            p.setPen(QColor("#777777"));
            p.setFont(QFont("monospace", 9));
            p.drawText(bar_x, H - margin_bot + 4, bar_w, margin_bot - 4,
                       Qt::AlignCenter,
                       QString("V%1").arg(i + 1));

            if (has_val) {
                float clamped = qMin(qMax(val, 0.0f), 2.0f);
                float y_frac  = clamped / 2.0f;
                int   bar_h   = static_cast<int>(plot_h * y_frac);
                int   bar_y   = margin_top + plot_h - bar_h;

                // Bar
                QColor bc(colors[i]);
                bc.setAlpha(180);
                p.fillRect(bar_x, bar_y, bar_w, bar_h, bc);

                // Border
                p.setPen(QPen(QColor(colors[i]), 1));
                p.drawRect(bar_x, bar_y, bar_w, bar_h);

                // Value above bar
                p.setPen(QColor(colors[i]));
                p.setFont(QFont("monospace", 9, QFont::Bold));
                p.drawText(bar_x - 2, qMax(margin_top, bar_y - 16),
                           bar_w + 4, 16,
                           Qt::AlignCenter,
                           QString::number(static_cast<double>(val), 'f', 2));
            } else {
                // No reading indicator
                p.setPen(QColor("#333333"));
                p.setFont(QFont("monospace", 9));
                p.drawText(bar_x, margin_top + plot_h / 2 - 8, bar_w, 16,
                           Qt::AlignCenter, "--");
            }
        }
    }

private:
    std::array<float,6> values_;
};

// Include the moc-generated code for SpectroPaint (since it's in a .cpp file)
#include "science_analysis_module.moc"

// ─────────────────────────────────────────────────────────────────────────────
// Resize event filter for font scaling
// ─────────────────────────────────────────────────────────────────────────────

class AnalysisScaleFilter : public QObject
{
public:
    explicit AnalysisScaleFilter(QWidget* target, int base_size, QObject* parent = nullptr)
        : QObject(parent), target_(target), base_size_(base_size)
    {}

    bool eventFilter(QObject* obj, QEvent* ev) override
    {
        if (ev->type() == QEvent::Resize) {
            auto* w = qobject_cast<QWidget*>(obj);
            if (w) {
                int sz = qMax(10, qMin(base_size_, w->height() / 30 + 10));
                QFont f = target_->font();
                f.setPointSize(sz);
                target_->setFont(f);
            }
        }
        return false;
    }

private:
    QWidget* target_;
    int      base_size_;
};

// ─────────────────────────────────────────────────────────────────────────────
// Style constants
// ─────────────────────────────────────────────────────────────────────────────

static const char* kDataCell =
    "QLabel { color: #ffffff; padding: 4px 6px; border: 1px solid #333333;"
    " background: #0a0a0a; }";
static const char* kDataCellGreen =
    "QLabel { color: #00ff88; padding: 4px 6px; border: 1px solid #00ff88;"
    " background: #001a0a; font-weight: bold; }";
static const char* kDataCellDim =
    "QLabel { color: #777777; padding: 4px 6px; border: 1px solid #222222;"
    " background: #050505; }";
static const char* kFlowGreen =
    "QLabel { color: #00ff88; font-weight: bold; padding: 4px 8px;"
    " border: 1px solid #00ff88; background: #001a0a; }";
static const char* kFlowDim =
    "QLabel { color: #555555; padding: 4px 8px;"
    " border: 1px solid #222222; background: #050505; }";

static QLabel* makeSectionHdr(const QString& text, const char* color)
{
    auto* lbl = new QLabel(text);
    lbl->setFont(QFont("monospace", theme::FontSize, QFont::Bold));
    lbl->setStyleSheet(
        QString("color: %1; padding: 6px 0 2px 0;"
                " border-bottom: 1px solid %2; margin-bottom: 4px;")
        .arg(color).arg(color));
    lbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    return lbl;
}

// ─────────────────────────────────────────────────────────────────────────────
// createWidget
// ─────────────────────────────────────────────────────────────────────────────

QWidget* ScienceAnalysisModule::createWidget(QWidget* parent)
{
    spectro_vals_.fill(std::numeric_limits<float>::quiet_NaN());

    auto* scroll = new QScrollArea(parent);
    scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setStyleSheet(
        "QScrollArea { border: none; background: #000000; }"
        "QScrollBar:vertical { width: 6px; background: #0a0a0a; }"
        "QScrollBar::handle:vertical { background: #333333; border-radius: 3px; }");

    auto* container = new QWidget();
    container->setStyleSheet(QString("background: %1;").arg(theme::Bg));
    container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    auto* root = new QVBoxLayout(container);
    root->setContentsMargins(12, 8, 12, 12);
    root->setSpacing(12);

    // ── ROW 0: SPECTROPHOTOMETER ─────────────────────────────────────────────
    root->addWidget(makeSectionHdr("Spectrophotometer", theme::Cyan));

    // 6 readout cells in 2 rows x 3 cols
    auto* spectro_grid = new QGridLayout();
    spectro_grid->setSpacing(4);
    for (int c = 0; c < 3; c++) spectro_grid->setColumnStretch(c, 1);
    spectro_grid->setRowStretch(0, 1);
    spectro_grid->setRowStretch(1, 1);

    for (int i = 0; i < 6; i++) {
        auto* cell = new QFrame();
        cell->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        cell->setStyleSheet(
            "QFrame { border: 1px solid #222222; background: #0a0a0a;"
            " border-radius: 4px; padding: 2px; }");

        auto* cl = new QVBoxLayout(cell);
        cl->setContentsMargins(6, 4, 6, 4);
        cl->setSpacing(2);

        auto* vial_lbl = new QLabel(QString("Vial %1").arg(i + 1));
        vial_lbl->setFont(QFont("monospace", theme::FontSizeSm, QFont::Bold));
        vial_lbl->setStyleSheet(
            QString("color: %1; border: none;").arg(theme::MotorColors[i]));
        vial_lbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        cl->addWidget(vial_lbl);

        spectro_lbls_[i] = new QLabel("--");
        spectro_lbls_[i]->setFont(QFont("monospace", theme::FontSize, QFont::Bold));
        spectro_lbls_[i]->setStyleSheet(kDataCellDim);
        spectro_lbls_[i]->setAlignment(Qt::AlignCenter);
        spectro_lbls_[i]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        cl->addWidget(spectro_lbls_[i]);

        spectro_grid->addWidget(cell, i / 3, i % 3);
    }
    root->addLayout(spectro_grid);

    // Bar chart
    spectro_chart_ = new SpectroPaint();
    spectro_chart_->setMinimumHeight(160);
    spectro_chart_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    root->addWidget(spectro_chart_, 1);

    // Clear All button
    auto* clear_row = new QHBoxLayout();
    clear_row->addStretch();
    auto* clear_btn = new QPushButton("Clear All");
    clear_btn->setFont(QFont("monospace", theme::FontSizeSm));
    clear_btn->setStyleSheet(
        "QPushButton { background: #1a0000; color: #ff4466; border: 1px solid #ff4466;"
        " border-radius: 6px; padding: 4px 12px; }");
    clear_btn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    QObject::connect(clear_btn, &QPushButton::clicked, [this]() {
        spectro_vals_.fill(std::numeric_limits<float>::quiet_NaN());
        if (spectro_chart_) spectro_chart_->clearAll();
        for (int i = 0; i < 6; i++) {
            if (spectro_lbls_[i]) {
                spectro_lbls_[i]->setText("--");
                spectro_lbls_[i]->setStyleSheet(kDataCellDim);
            }
        }
    });
    clear_row->addWidget(clear_btn);
    root->addLayout(clear_row);

    // ── ROW 1: NPK PROBE ─────────────────────────────────────────────────────
    root->addWidget(makeSectionHdr("NPK Probe", theme::Green));

    auto* npk_row = new QHBoxLayout();
    npk_row->setSpacing(8);

    // N, P, K large displays
    struct NpkCell { const char* element; const char* color; QLabel** lbl_ptr; };
    NpkCell npk_cells[] = {
        { "N",  theme::Green,  &npk_n_lbl_ },
        { "P",  theme::Yellow, &npk_p_lbl_ },
        { "K",  theme::Cyan,   &npk_k_lbl_ },
    };

    for (auto& nc : npk_cells) {
        auto* frame = new QFrame();
        frame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        frame->setStyleSheet(
            QString("QFrame { border: 1px solid #333333; background: #0a0a0a;"
                    " border-radius: 6px; }"));

        auto* fl = new QVBoxLayout(frame);
        fl->setContentsMargins(8, 6, 8, 6);
        fl->setSpacing(2);

        auto* elem_lbl = new QLabel(nc.element);
        elem_lbl->setFont(QFont("monospace", 22, QFont::Bold));
        elem_lbl->setStyleSheet(
            QString("color: %1; border: none; padding: 0;").arg(nc.color));
        elem_lbl->setAlignment(Qt::AlignCenter);
        elem_lbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        fl->addWidget(elem_lbl);

        *nc.lbl_ptr = new QLabel("-- mg/kg");
        (*nc.lbl_ptr)->setFont(QFont("monospace", theme::FontSize, QFont::Bold));
        (*nc.lbl_ptr)->setStyleSheet(
            QString("color: %1; border: none; padding: 2px;").arg(nc.color));
        (*nc.lbl_ptr)->setAlignment(Qt::AlignCenter);
        (*nc.lbl_ptr)->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        fl->addWidget(*nc.lbl_ptr);

        npk_row->addWidget(frame, 1);
    }
    root->addLayout(npk_row);

    // Record button + status
    auto* rec_row = new QHBoxLayout();
    npk_rec_btn_ = new QPushButton("⏺ Record");
    npk_rec_btn_->setFont(QFont("monospace", theme::FontSize, QFont::Bold));
    npk_rec_btn_->setStyleSheet(
        "QPushButton { background: #0a0a0a; color: #ffffff;"
        " border: 1px solid #444444; border-radius: 6px; padding: 6px 16px; }");
    npk_rec_btn_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    QObject::connect(npk_rec_btn_, &QPushButton::clicked, [this]() {
        toggleRecording();
    });
    rec_row->addWidget(npk_rec_btn_);

    npk_status_ = new QLabel("Not recording");
    npk_status_->setFont(QFont("monospace", theme::FontSizeSm));
    npk_status_->setStyleSheet(
        QString("color: %1; padding: 0 8px;").arg(theme::TextDim));
    npk_status_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    rec_row->addWidget(npk_status_);
    root->addLayout(rec_row);

    // ── ROW 2: FLUOROMETER + GAS SENSOR (side by side) ───────────────────────
    auto* fg_row = new QHBoxLayout();
    fg_row->setSpacing(8);

    // Fluorometer
    auto* fluoro_frame = new QFrame();
    fluoro_frame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    fluoro_frame->setStyleSheet(
        "QFrame { border: 1px solid #333333; background: #050505; border-radius: 6px; }");
    auto* fluoro_layout = new QVBoxLayout(fluoro_frame);
    fluoro_layout->setContentsMargins(8, 8, 8, 8);
    fluoro_layout->setSpacing(4);

    auto* fluoro_hdr = new QLabel("Fluorometer");
    fluoro_hdr->setFont(QFont("monospace", theme::FontSize, QFont::Bold));
    fluoro_hdr->setStyleSheet(
        QString("color: %1; border: none; padding: 0;").arg(theme::Cyan));
    fluoro_hdr->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    fluoro_layout->addWidget(fluoro_hdr);

    fluoro_val_lbl_ = new QLabel("-- arb");
    fluoro_val_lbl_->setFont(QFont("monospace", 20, QFont::Bold));
    fluoro_val_lbl_->setStyleSheet(kDataCell);
    fluoro_val_lbl_->setAlignment(Qt::AlignCenter);
    fluoro_val_lbl_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    fluoro_layout->addWidget(fluoro_val_lbl_);

    fluoro_bar_ = new QProgressBar();
    fluoro_bar_->setRange(0, 1000);
    fluoro_bar_->setValue(0);
    fluoro_bar_->setTextVisible(false);
    fluoro_bar_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    fluoro_bar_->setStyleSheet(
        QString("QProgressBar { background: #0a0a0a; border: 1px solid #333333;"
                " border-radius: 4px; }"
                "QProgressBar::chunk { background: %1; border-radius: 3px; }").arg(theme::Cyan));
    fluoro_layout->addWidget(fluoro_bar_);

    fg_row->addWidget(fluoro_frame, 1);

    // Gas Sensor
    auto* gas_frame = new QFrame();
    gas_frame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    gas_frame->setStyleSheet(
        "QFrame { border: 1px solid #333333; background: #050505; border-radius: 6px; }");
    auto* gas_layout = new QVBoxLayout(gas_frame);
    gas_layout->setContentsMargins(8, 8, 8, 8);
    gas_layout->setSpacing(4);

    auto* gas_hdr = new QLabel("Gas Sensor");
    gas_hdr->setFont(QFont("monospace", theme::FontSize, QFont::Bold));
    gas_hdr->setStyleSheet(
        QString("color: %1; border: none; padding: 0;").arg(theme::Yellow));
    gas_hdr->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    gas_layout->addWidget(gas_hdr);

    gas_val_lbl_ = new QLabel("-- ppm");
    gas_val_lbl_->setFont(QFont("monospace", 20, QFont::Bold));
    gas_val_lbl_->setStyleSheet(kDataCell);
    gas_val_lbl_->setAlignment(Qt::AlignCenter);
    gas_val_lbl_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    gas_layout->addWidget(gas_val_lbl_);

    gas_avg_lbl_ = new QLabel("avg (30s): -- ppm");
    gas_avg_lbl_->setFont(QFont("monospace", theme::FontSizeSm));
    gas_avg_lbl_->setStyleSheet(kDataCellDim);
    gas_avg_lbl_->setAlignment(Qt::AlignCenter);
    gas_avg_lbl_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    gas_layout->addWidget(gas_avg_lbl_);

    fg_row->addWidget(gas_frame, 1);
    root->addLayout(fg_row);

    // ── ROW 3: FLOW SENSORS ───────────────────────────────────────────────────
    root->addWidget(makeSectionHdr("Flow Sensors", theme::TextDim));

    auto* flow_row = new QHBoxLayout();
    flow_row->setSpacing(8);

    auto* flow1_frame = new QFrame();
    flow1_frame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    flow1_frame->setStyleSheet(
        "QFrame { border: 1px solid #222222; background: #050505; border-radius: 4px; }");
    auto* f1l = new QHBoxLayout(flow1_frame);
    f1l->setContentsMargins(8, 6, 8, 6);
    auto* f1_name = new QLabel("OSF1:");
    f1_name->setFont(QFont("monospace", theme::FontSize, QFont::Bold));
    f1_name->setStyleSheet("color: #ffffff; border: none;");
    f1_name->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    f1l->addWidget(f1_name);
    flow1_lbl_ = new QLabel("NO DATA ○");
    flow1_lbl_->setFont(QFont("monospace", theme::FontSize));
    flow1_lbl_->setStyleSheet(kFlowDim);
    flow1_lbl_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    f1l->addWidget(flow1_lbl_);
    flow_row->addWidget(flow1_frame, 1);

    auto* flow2_frame = new QFrame();
    flow2_frame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    flow2_frame->setStyleSheet(
        "QFrame { border: 1px solid #222222; background: #050505; border-radius: 4px; }");
    auto* f2l = new QHBoxLayout(flow2_frame);
    f2l->setContentsMargins(8, 6, 8, 6);
    auto* f2_name = new QLabel("OSF2:");
    f2_name->setFont(QFont("monospace", theme::FontSize, QFont::Bold));
    f2_name->setStyleSheet("color: #ffffff; border: none;");
    f2_name->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    f2l->addWidget(f2_name);
    flow2_lbl_ = new QLabel("NO DATA ○");
    flow2_lbl_->setFont(QFont("monospace", theme::FontSize));
    flow2_lbl_->setStyleSheet(kFlowDim);
    flow2_lbl_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    f2l->addWidget(flow2_lbl_);
    flow_row->addWidget(flow2_frame, 1);

    root->addLayout(flow_row);
    root->addStretch();

    scroll->setWidget(container);
    return scroll;
}

// ─────────────────────────────────────────────────────────────────────────────
// setNode / start / stop
// ─────────────────────────────────────────────────────────────────────────────

void ScienceAnalysisModule::setNode(rclcpp::Node::SharedPtr node)
{
    node_ = node;
    sub_ = node_->create_subscription<rover_msgs::msg::ScienceSensorData>(
        "/science/sensor_data",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        [this](const rover_msgs::msg::ScienceSensorData::SharedPtr msg) {
            onSensorData(msg);
        });
}

void ScienceAnalysisModule::start()
{
    // Nothing to start — subscription is created in setNode
}

void ScienceAnalysisModule::stop()
{
    // Close NPK recording file if open
    if (npk_recording_) {
        npk_recording_ = false;
        if (npk_stream_) {
            delete npk_stream_;
            npk_stream_ = nullptr;
        }
        if (npk_file_) {
            npk_file_->close();
            delete npk_file_;
            npk_file_ = nullptr;
        }
        if (npk_rec_btn_) {
            npk_rec_btn_->setText("⏺ Record");
            npk_rec_btn_->setStyleSheet(
                "QPushButton { background: #0a0a0a; color: #ffffff;"
                " border: 1px solid #444444; border-radius: 6px; padding: 6px 16px; }");
        }
        if (npk_status_)
            npk_status_->setText("Not recording");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// onSensorData
// ─────────────────────────────────────────────────────────────────────────────

void ScienceAnalysisModule::onSensorData(
    const rover_msgs::msg::ScienceSensorData::SharedPtr msg)
{
    // ── Spectro ──────────────────────────────────────────────────────────────
    for (int i = 0; i < 6; i++) {
        float val = msg->spectro_absorbance[i];
        if (msg->spectro_ready) {
            spectro_vals_[i] = val;
        }
        if (spectro_lbls_[i]) {
            if (msg->spectro_ready) {
                spectro_lbls_[i]->setText(
                    QString("%1 AU").arg(static_cast<double>(val), 0, 'f', 3));
                spectro_lbls_[i]->setStyleSheet(kDataCellGreen);
            } else if (!std::isnan(spectro_vals_[i])) {
                spectro_lbls_[i]->setText(
                    QString("%1 AU").arg(
                        static_cast<double>(spectro_vals_[i]), 0, 'f', 3));
                spectro_lbls_[i]->setStyleSheet(kDataCell);
            }
        }
    }
    if (spectro_chart_ && msg->spectro_ready) {
        spectro_chart_->setValues(spectro_vals_);
    }

    // ── NPK ──────────────────────────────────────────────────────────────────
    if (npk_n_lbl_)
        npk_n_lbl_->setText(
            QString("%1 mg/kg").arg(
                static_cast<double>(msg->npk_nitrogen), 0, 'f', 1));
    if (npk_p_lbl_)
        npk_p_lbl_->setText(
            QString("%1 mg/kg").arg(
                static_cast<double>(msg->npk_phosphorus), 0, 'f', 1));
    if (npk_k_lbl_)
        npk_k_lbl_->setText(
            QString("%1 mg/kg").arg(
                static_cast<double>(msg->npk_potassium), 0, 'f', 1));

    if (npk_recording_) {
        appendNpkRecord(msg->npk_nitrogen,
                        msg->npk_phosphorus,
                        msg->npk_potassium);
    }

    // ── Fluorometer ──────────────────────────────────────────────────────────
    float fluoro = msg->fluorometer_value;
    if (fluoro_val_lbl_)
        fluoro_val_lbl_->setText(
            QString("%1 arb").arg(static_cast<double>(fluoro), 0, 'f', 1));
    if (fluoro_bar_)
        fluoro_bar_->setValue(
            qMin(1000, static_cast<int>(fluoro)));

    // ── Gas sensor ───────────────────────────────────────────────────────────
    float gas = msg->gas_sensor_value;
    if (gas_val_lbl_)
        gas_val_lbl_->setText(
            QString("%1 ppm").arg(static_cast<double>(gas), 0, 'f', 1));

    gas_history_.push_back(gas);
    while (static_cast<int>(gas_history_.size()) > 30)
        gas_history_.pop_front();

    if (gas_avg_lbl_ && !gas_history_.empty()) {
        float sum = std::accumulate(gas_history_.begin(), gas_history_.end(), 0.0f);
        float avg = sum / static_cast<float>(gas_history_.size());
        gas_avg_lbl_->setText(
            QString("avg (%1s): %2 ppm")
            .arg(gas_history_.size())
            .arg(static_cast<double>(avg), 0, 'f', 1));
    }

    // ── Flow sensors ─────────────────────────────────────────────────────────
    bool flow1 = msg->flow_sensor_1_v > 0.5f;
    bool flow2 = msg->flow_sensor_2_v > 0.5f;

    if (flow1_lbl_) {
        flow1_lbl_->setText(
            flow1
            ? QString("FLOWING ●  (%1 V)")
              .arg(static_cast<double>(msg->flow_sensor_1_v), 0, 'f', 2)
            : QString("NO FLOW ○  (%1 V)")
              .arg(static_cast<double>(msg->flow_sensor_1_v), 0, 'f', 2));
        flow1_lbl_->setStyleSheet(flow1 ? kFlowGreen : kFlowDim);
    }
    if (flow2_lbl_) {
        flow2_lbl_->setText(
            flow2
            ? QString("FLOWING ●  (%1 V)")
              .arg(static_cast<double>(msg->flow_sensor_2_v), 0, 'f', 2)
            : QString("NO FLOW ○  (%1 V)")
              .arg(static_cast<double>(msg->flow_sensor_2_v), 0, 'f', 2));
        flow2_lbl_->setStyleSheet(flow2 ? kFlowGreen : kFlowDim);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// NPK Recording
// ─────────────────────────────────────────────────────────────────────────────

void ScienceAnalysisModule::toggleRecording()
{
    if (npk_recording_) {
        // Stop recording
        npk_recording_ = false;
        if (npk_stream_) {
            delete npk_stream_;
            npk_stream_ = nullptr;
        }
        if (npk_file_) {
            npk_file_->close();
            delete npk_file_;
            npk_file_ = nullptr;
        }
        if (npk_rec_btn_) {
            npk_rec_btn_->setText("⏺ Record");
            npk_rec_btn_->setStyleSheet(
                "QPushButton { background: #0a0a0a; color: #ffffff;"
                " border: 1px solid #444444; border-radius: 6px; padding: 6px 16px; }");
        }
        if (npk_status_)
            npk_status_->setText("Not recording");
    } else {
        // Start recording
        QString timestamp =
            QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss");
        QString home = QDir::homePath();
        QString filename =
            home + "/npk_log_" + timestamp + ".csv";

        npk_file_ = new QFile(filename);
        if (!npk_file_->open(QIODevice::WriteOnly | QIODevice::Text)) {
            delete npk_file_;
            npk_file_ = nullptr;
            if (npk_status_)
                npk_status_->setText("ERROR: could not open file");
            return;
        }
        npk_stream_ = new QTextStream(npk_file_);
        (*npk_stream_) << "timestamp_iso,nitrogen_mg_kg,phosphorus_mg_kg,potassium_mg_kg\n";
        npk_stream_->flush();

        npk_recording_ = true;

        if (npk_rec_btn_) {
            npk_rec_btn_->setText("⏹ Stop");
            npk_rec_btn_->setStyleSheet(
                "QPushButton { background: #330000; color: #ff4466;"
                " border: 1px solid #ff4466; border-radius: 6px;"
                " padding: 6px 16px; font-weight: bold; }");
        }
        if (npk_status_)
            npk_status_->setText(
                QString("Recording → ~/npk_log_%1.csv").arg(timestamp));
    }
}

void ScienceAnalysisModule::appendNpkRecord(float n, float p, float k)
{
    if (!npk_stream_) return;
    QString ts = QDateTime::currentDateTime().toString(Qt::ISODate);
    (*npk_stream_) << ts << ","
                   << QString::number(static_cast<double>(n), 'f', 2) << ","
                   << QString::number(static_cast<double>(p), 'f', 2) << ","
                   << QString::number(static_cast<double>(k), 'f', 2) << "\n";
    npk_stream_->flush();
}

// ─────────────────────────────────────────────────────────────────────────────

PLUGINLIB_EXPORT_CLASS(ScienceAnalysisModule, rover_hmi_core::GuiModule)
