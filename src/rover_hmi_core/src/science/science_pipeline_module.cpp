#include "science_pipeline_module.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QScrollArea>
#include <QSizePolicy>
#include <QFont>
#include <QMessageBox>

#include <pluginlib/class_list_macros.hpp>
#include <rover_hmi_core/catppuccin.h>

// ─────────────────────────────────────────────────────────────────────────────
// Resize event filter — scales fonts proportionally when the panel resizes
// ─────────────────────────────────────────────────────────────────────────────

class PipelineScaleFilter : public QObject
{
public:
    explicit PipelineScaleFilter(QWidget* target, int base_size, QObject* parent = nullptr)
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
// Style helpers
// ─────────────────────────────────────────────────────────────────────────────

static const char* kStepWaiting =
    "QFrame { border: 1px solid #333333; background: #000000; border-radius: 6px; }";
static const char* kStepActive =
    "QFrame { border: 2px solid #ffcc00; background: #0a0800; border-radius: 6px; }";
static const char* kStepDone =
    "QFrame { border: 1px solid #00ff88; background: #001a0a; border-radius: 6px; }";

static const char* kNavBtn =
    "QPushButton { background: #0a0a0a; color: #ffffff; border: 1px solid #333333;"
    " border-radius: 6px; padding: 8px 16px; font-weight: bold; }";
static const char* kNavBtnDisabled =
    "QPushButton { background: #050505; color: #333333; border: 1px solid #1a1a1a;"
    " border-radius: 6px; padding: 8px 16px; }";
static const char* kNextBtn =
    "QPushButton { background: #002211; color: #00ff88; border: 1px solid #00ff88;"
    " border-radius: 6px; padding: 8px 16px; font-weight: bold; }";
static const char* kResetBtn =
    "QPushButton { background: #221100; color: #ffcc00; border: 1px solid #ffcc00;"
    " border-radius: 6px; padding: 8px 16px; font-weight: bold; }";

static const char* kSensorCell =
    "QLabel { color: #ffffff; padding: 4px 6px; border: 1px solid #333333;"
    " background: #0a0a0a; }";
static const char* kSensorGreen =
    "QLabel { color: #00ff88; padding: 4px 6px; border: 1px solid #00ff88;"
    " background: #001a0a; font-weight: bold; }";
static const char* kSensorDim =
    "QLabel { color: #777777; padding: 4px 6px; border: 1px solid #333333;"
    " background: #0a0a0a; }";

// ─────────────────────────────────────────────────────────────────────────────
// Helper: section header
// ─────────────────────────────────────────────────────────────────────────────

static QLabel* makeHeader(const QString& text, const char* color = "#ffffff")
{
    auto* lbl = new QLabel(text);
    lbl->setStyleSheet(QString(
        "color: %1; font-size: 14px; font-weight: bold;"
        " padding: 4px 0; letter-spacing: 1px;").arg(color));
    lbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    return lbl;
}

static QLabel* makeBodyText(const QString& text)
{
    auto* lbl = new QLabel(text);
    lbl->setStyleSheet(
        "color: #777777; font-style: italic; padding: 2px 0;");
    lbl->setWordWrap(true);
    lbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    return lbl;
}

static const char* kCheckOff =
    "QPushButton { text-align: left; background: transparent; border: none;"
    " color: #aaaaaa; padding: 2px 4px; }";
static const char* kCheckOn =
    "QPushButton { text-align: left; background: transparent; border: none;"
    " color: #00ff88; padding: 2px 4px; }";

static const char* kVialOff =
    "QPushButton { background: #0a0a0a; color: #777777;"
    " border: 2px solid #333333; border-radius: 8px; padding: 10px 4px; }";
static const char* kVialOn =
    "QPushButton { background: #001a0a; color: #00ff88;"
    " border: 2px solid #00ff88; border-radius: 8px;"
    " padding: 10px 4px; font-weight: bold; }";

static QLabel* makeHint(const QString& text)
{
    auto* lbl = new QLabel(text);
    lbl->setStyleSheet(
        "color: #555555; font-style: italic; font-size: 12px;"
        " padding: 4px 0;");
    lbl->setWordWrap(true);
    lbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    return lbl;
}

// ─────────────────────────────────────────────────────────────────────────────
// createWidget
// ─────────────────────────────────────────────────────────────────────────────

QWidget* SciencePipelineModule::createWidget(QWidget* parent)
{
    auto* root_widget = new QWidget(parent);
    root_widget->setStyleSheet(QString("background: %1;").arg(theme::Bg));
    root_widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    auto* root = new QVBoxLayout(root_widget);
    root->setContentsMargins(8, 8, 8, 8);
    root->setSpacing(8);

    // ── TOP STEP BAR ─────────────────────────────────────────────────────────
    auto* step_bar = new QHBoxLayout();
    step_bar->setSpacing(4);

    const char* step_names[] = { "DRILL", "COLLECT", "SEPARATE", "ANALYZE" };

    for (int i = 0; i < 4; i++) {
        auto* frame = new QFrame();
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        step_frames_[i] = frame;

        auto* vl = new QVBoxLayout(frame);
        vl->setContentsMargins(8, 8, 8, 8);
        vl->setSpacing(4);

        auto* num_lbl = new QLabel(QString::number(i + 1));
        num_lbl->setAlignment(Qt::AlignCenter);
        num_lbl->setFont(QFont("monospace", 20, QFont::Bold));
        num_lbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        vl->addWidget(num_lbl);

        auto* name_lbl = new QLabel(step_names[i]);
        name_lbl->setAlignment(Qt::AlignCenter);
        name_lbl->setFont(QFont("monospace", theme::FontSize, QFont::Bold));
        name_lbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        step_name_lbls_[i] = name_lbl;
        vl->addWidget(name_lbl);

        auto* dot_lbl = new QLabel("○ waiting");
        dot_lbl->setAlignment(Qt::AlignCenter);
        dot_lbl->setFont(QFont("monospace", theme::FontSize));
        dot_lbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        step_dot_lbls_[i] = dot_lbl;
        vl->addWidget(dot_lbl);

        step_bar->addWidget(frame, 1);
    }

    root->addLayout(step_bar);

    // ── DETAIL STACK ─────────────────────────────────────────────────────────
    detail_stack_ = new QStackedWidget();
    detail_stack_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    detail_stack_->setStyleSheet(
        "QStackedWidget { background: #050505; border: 1px solid #222222;"
        " border-radius: 6px; }");

    // --- PAGE 0: DRILL -------------------------------------------------------
    {
        auto* page = new QWidget();
        page->setStyleSheet(QString("background: %1;").arg(theme::Bg));
        page->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        auto* vl = new QVBoxLayout(page);
        vl->setContentsMargins(12, 12, 12, 12);
        vl->setSpacing(8);

        vl->addWidget(makeHeader("STEP 1 — DRILL", theme::Yellow));
        vl->addWidget(makeBodyText(
            "Auger drill churns soil to prepare it for collection."));

        // Sensor area
        auto* sensor_grid = new QGridLayout();
        sensor_grid->setSpacing(4);
        auto* slbl = new QLabel("Ultrasonic Distance:");
        slbl->setFont(QFont("monospace", theme::FontSize));
        slbl->setStyleSheet(QString("color: %1;").arg(theme::TextDim));
        slbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sensor_grid->addWidget(slbl, 0, 0);
        sensor_grid->setColumnStretch(0, 1);
        sensor_grid->setColumnStretch(1, 1);

        ultrasonic_lbl_ = new QLabel("-- in");
        ultrasonic_lbl_->setFont(QFont("monospace", theme::FontSize, QFont::Bold));
        ultrasonic_lbl_->setStyleSheet(kSensorCell);
        ultrasonic_lbl_->setAlignment(Qt::AlignCenter);
        ultrasonic_lbl_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sensor_grid->addWidget(ultrasonic_lbl_, 0, 1);

        vl->addLayout(sensor_grid);

        vl->addWidget(makeHeader("Checklist:", theme::Cyan));
        vl->addWidget(makeCheck(0, 0, "Position rover over sample site"));
        vl->addWidget(makeCheck(0, 1, "Activate drill motor (in Science Control panel)"));
        vl->addWidget(makeCheck(0, 2, "Lower drill to target depth (≥ 6.0 in)"));

        vl->addStretch();
        vl->addWidget(makeHint("Auto-advances when target depth is reached"));

        detail_stack_->addWidget(page);
    }

    // --- PAGE 1: COLLECT -----------------------------------------------------
    {
        auto* page = new QWidget();
        page->setStyleSheet(QString("background: %1;").arg(theme::Bg));
        page->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        auto* vl = new QVBoxLayout(page);
        vl->setContentsMargins(12, 12, 12, 12);
        vl->setSpacing(8);

        vl->addWidget(makeHeader("STEP 2 — COLLECT", theme::Yellow));
        vl->addWidget(makeBodyText(
            "Vacuum draws churned soil through the collection tube."));

        vl->addWidget(makeHeader("Checklist:", theme::Cyan));
        vl->addWidget(makeCheck(1, 0, "Enable vacuum (in Science Control panel)"));
        vl->addWidget(makeCheck(1, 1, "Open collection servo (dispense)"));
        vl->addWidget(makeCheck(1, 2, "Sample collected (visual check)"));

        vl->addStretch();
        vl->addWidget(makeHint("No flow sensors — confirm collection visually"));

        detail_stack_->addWidget(page);
    }

    // --- PAGE 2: SEPARATE ----------------------------------------------------
    {
        auto* page = new QWidget();
        page->setStyleSheet(QString("background: %1;").arg(theme::Bg));
        page->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        auto* vl = new QVBoxLayout(page);
        vl->setContentsMargins(12, 12, 12, 12);
        vl->setSpacing(8);

        vl->addWidget(makeHeader("STEP 3 — SEPARATE", theme::Yellow));
        vl->addWidget(makeBodyText(
            "3-way carousel distributes soil into 3 vials via the drop servo."));

        // Vial grid
        vl->addWidget(makeHeader("Vial Fill Status:", theme::Cyan));

        auto* vial_grid = new QGridLayout();
        vial_grid->setSpacing(6);
        for (int i = 0; i < kNumVials; i++)
            vial_grid->setColumnStretch(i, 1);
        vial_grid->setRowStretch(0, 1);

        for (int i = 0; i < kNumVials; i++) {
            vial_btns_[i] = new QPushButton(QString("○ Vial %1").arg(i + 1));
            vial_btns_[i]->setFont(QFont("monospace", theme::FontSize));
            vial_btns_[i]->setStyleSheet(kVialOff);
            vial_btns_[i]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
            int idx = i;
            QObject::connect(vial_btns_[i], &QPushButton::clicked, [this, idx]() {
                setVial(idx, !vials_filled_[idx]);
            });
            vial_grid->addWidget(vial_btns_[i], i / 3, i % 3);
        }
        vl->addLayout(vial_grid);

        vials_count_lbl_ = new QLabel("0 / 6 vials filled");
        vials_count_lbl_->setFont(QFont("monospace", theme::FontSize));
        vials_count_lbl_->setStyleSheet(
            QString("color: %1; font-weight: bold; padding: 4px 0;").arg(theme::Cyan));
        vials_count_lbl_->setAlignment(Qt::AlignCenter);
        vials_count_lbl_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        vl->addWidget(vials_count_lbl_);

        vl->addWidget(makeHeader("Checklist:", theme::Cyan));
        vl->addWidget(makeCheck(2, 0, "Rotate carousel to index 1"));
        vl->addWidget(makeCheck(2, 1, "Open hinge servo to drop sample"));
        vl->addWidget(makeCheck(2, 2, "Fill all 6 vials (auto as carousel visits 1–6)"));

        vl->addStretch();
        vl->addWidget(makeHint("Auto-advances when all 6 vials are filled"));

        detail_stack_->addWidget(page);
    }

    // --- PAGE 3: ANALYZE -----------------------------------------------------
    {
        auto* page = new QWidget();
        page->setStyleSheet(QString("background: %1;").arg(theme::Bg));
        page->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        auto* vl = new QVBoxLayout(page);
        vl->setContentsMargins(12, 12, 12, 12);
        vl->setSpacing(8);

        vl->addWidget(makeHeader("STEP 4 — ANALYZE", theme::Yellow));
        vl->addWidget(makeBodyText(
            "Spectrophotometer measures absorbance for each vial."));

        auto* spectro_grid = new QGridLayout();
        spectro_grid->setSpacing(4);
        for (int i = 0; i < 3; i++) {
            spectro_grid->setColumnStretch(i * 2, 1);
            spectro_grid->setColumnStretch(i * 2 + 1, 1);
        }

        for (int i = 0; i < kNumVials; i++) {
            auto* vlbl = new QLabel(QString("Vial %1:").arg(i + 1));
            vlbl->setFont(QFont("monospace", theme::FontSize));
            vlbl->setStyleSheet(
                QString("color: %1;").arg(theme::MotorColors[i]));
            vlbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

            spectro_lbls_[i] = new QLabel("--");
            spectro_lbls_[i]->setFont(QFont("monospace", theme::FontSize, QFont::Bold));
            spectro_lbls_[i]->setStyleSheet(kSensorDim);
            spectro_lbls_[i]->setAlignment(Qt::AlignCenter);
            spectro_lbls_[i]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

            spectro_grid->addWidget(vlbl,           i / 3, (i % 3) * 2);
            spectro_grid->addWidget(spectro_lbls_[i], i / 3, (i % 3) * 2 + 1);
        }

        vl->addWidget(makeHeader("Spectro Absorbance:", theme::Cyan));
        vl->addLayout(spectro_grid);

        vl->addWidget(makeHeader("Checklist:", theme::Cyan));
        vl->addWidget(makeCheck(3, 0, "Power spectrometer (in Science Control panel)"));
        vl->addWidget(makeCheck(3, 1, "Spectro reading ready"));
        vl->addWidget(makeCheck(3, 2, "All 6 vials read"));

        vl->addStretch();
        vl->addWidget(makeHint("Pipeline completes when all 6 vials are read"));

        detail_stack_->addWidget(page);
    }

    root->addWidget(detail_stack_, 1);

    // ── NAVIGATION BUTTONS ───────────────────────────────────────────────────
    auto* nav_row = new QHBoxLayout();
    nav_row->setSpacing(8);

    back_btn_ = new QPushButton("← Back");
    back_btn_->setFont(QFont("monospace", theme::FontSize, QFont::Bold));
    back_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    nav_row->addWidget(back_btn_);
    QObject::connect(back_btn_, &QPushButton::clicked, [this]() { retreatStep(); });

    reset_btn_ = new QPushButton("Reset Pipeline");
    reset_btn_->setFont(QFont("monospace", theme::FontSize, QFont::Bold));
    reset_btn_->setStyleSheet(kResetBtn);
    reset_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    nav_row->addWidget(reset_btn_);
    QObject::connect(reset_btn_, &QPushButton::clicked, [this]() { resetPipeline(); });

    next_btn_ = new QPushButton("Mark Done → Next ▶");
    next_btn_->setFont(QFont("monospace", theme::FontSize, QFont::Bold));
    next_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    nav_row->addWidget(next_btn_);
    QObject::connect(next_btn_, &QPushButton::clicked, [this]() { advanceStep(); });

    root->addLayout(nav_row);

    // Initial display
    updateStepDisplay();

    return root_widget;
}

// ─────────────────────────────────────────────────────────────────────────────
// updateStepDisplay
// ─────────────────────────────────────────────────────────────────────────────

void SciencePipelineModule::updateStepDisplay()
{
    if (!detail_stack_) return;
    detail_stack_->setCurrentIndex(current_step_);

    for (int i = 0; i < 4; i++) {
        if (!step_frames_[i]) continue;

        if (i < current_step_ || steps_done_[i]) {
            // Done
            step_frames_[i]->setStyleSheet(kStepDone);
            step_dot_lbls_[i]->setText("✓ done");
            step_dot_lbls_[i]->setStyleSheet(
                QString("color: %1; font-weight: bold;").arg(theme::Green));
            step_name_lbls_[i]->setStyleSheet(
                QString("color: %1; font-weight: bold;").arg(theme::Green));
        } else if (i == current_step_) {
            // Active
            step_frames_[i]->setStyleSheet(kStepActive);
            step_dot_lbls_[i]->setText("● active");
            step_dot_lbls_[i]->setStyleSheet(
                QString("color: %1; font-weight: bold;").arg(theme::Yellow));
            step_name_lbls_[i]->setStyleSheet(
                QString("color: %1; font-weight: bold;").arg(theme::Yellow));
        } else {
            // Waiting
            step_frames_[i]->setStyleSheet(kStepWaiting);
            step_dot_lbls_[i]->setText("○ waiting");
            step_dot_lbls_[i]->setStyleSheet(
                QString("color: %1;").arg(theme::TextDim));
            step_name_lbls_[i]->setStyleSheet(
                QString("color: %1;").arg(theme::TextDim));
        }
    }

    // Update nav buttons
    if (back_btn_) {
        bool can_back = (current_step_ > 0);
        back_btn_->setEnabled(can_back);
        back_btn_->setStyleSheet(can_back ? kNavBtn : kNavBtnDisabled);
    }

    if (next_btn_) {
        bool pipeline_done = (current_step_ == 3 && steps_done_[3]);
        if (pipeline_done) {
            next_btn_->setText("Pipeline Complete ✓");
            next_btn_->setEnabled(false);
            next_btn_->setStyleSheet(
                "QPushButton { background: #001a0a; color: #00ff88;"
                " border: 1px solid #00ff88; border-radius: 6px;"
                " padding: 8px 16px; font-weight: bold; }");
        } else {
            next_btn_->setText("Mark Done → Next ▶");
            next_btn_->setEnabled(true);
            next_btn_->setStyleSheet(kNextBtn);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Navigation helpers
// ─────────────────────────────────────────────────────────────────────────────

void SciencePipelineModule::advanceStep()
{
    steps_done_[current_step_] = true;
    if (current_step_ < 3) {
        current_step_++;
    }
    updateStepDisplay();
}

void SciencePipelineModule::retreatStep()
{
    if (current_step_ > 0) {
        steps_done_[current_step_] = false;
        current_step_--;
        steps_done_[current_step_] = false;
        updateStepDisplay();
    }
}

void SciencePipelineModule::resetPipeline()
{
    auto* parent_widget = (back_btn_ ? back_btn_->parentWidget() : nullptr);
    int ret = QMessageBox::question(
        parent_widget,
        "Reset Pipeline",
        "Reset all pipeline steps to the beginning?\n"
        "This will also clear vial filled state.",
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);

    if (ret != QMessageBox::Yes) return;

    current_step_ = 0;
    for (int i = 0; i < 4; i++) steps_done_[i] = false;
    for (int i = 0; i < kNumVials; i++) setVial(i, false);
    for (auto& row : checks_)
        for (auto* c : row)
            if (c) c->setChecked(false);

    updateStepDisplay();
}

// ─────────────────────────────────────────────────────────────────────────────
// setNode
// ─────────────────────────────────────────────────────────────────────────────

void SciencePipelineModule::setNode(rclcpp::Node::SharedPtr node)
{
    node_ = node;
    sensor_sub_ = node_->create_subscription<rover_msgs::msg::ScienceSensorData>(
        "/science/sensor_data",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        [this](const rover_msgs::msg::ScienceSensorData::SharedPtr msg) {
            onSensorData(msg);
        });
    cmd_sub_ = node_->create_subscription<rover_msgs::msg::ScienceModule>(
        "/science/command",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        [this](const rover_msgs::msg::ScienceModule::SharedPtr msg) {
            onCommand(msg);
        });
}

// ─────────────────────────────────────────────────────────────────────────────
// onCommand — operator actions in Science Control auto-check checklist items
// ─────────────────────────────────────────────────────────────────────────────

void SciencePipelineModule::onCommand(const rover_msgs::msg::ScienceModule::SharedPtr msg)
{
    if (msg->drilldir != 0) {
        setCheck(0, 0, true);  // positioning implied by starting the drill
        setCheck(0, 1, true);
    }
    if (msg->vacuumstatus == 1) setCheck(1, 0, true);
    if (msg->largestatus == 1 || msg->smallstatus == 1) setCheck(1, 1, true);

    if (msg->carouselindex >= 1) setCheck(2, 0, true);
    if (msg->carouselindex >= 1 && msg->carouselindex <= 6)
        setVial(msg->carouselindex - 1, true);
    if (msg->largestatus == 1 || msg->smallstatus == 1) setCheck(2, 1, true);

    if (msg->spectrostatus == 1) setCheck(3, 0, true);
}

// ─────────────────────────────────────────────────────────────────────────────
// onSensorData
// ─────────────────────────────────────────────────────────────────────────────

void SciencePipelineModule::onSensorData(
    const rover_msgs::msg::ScienceSensorData::SharedPtr msg)
{
    // Ultrasonic
    if (ultrasonic_lbl_) {
        ultrasonic_lbl_->setText(
            QString("%1 in").arg(static_cast<double>(msg->ultrasonic_distance_in), 0, 'f', 2));
        ultrasonic_lbl_->setStyleSheet(kSensorCell);
    }
    if (msg->ultrasonic_distance_in >= 6.0f) setCheck(0, 2, true);

    // Spectro
    for (int i = 0; i < kNumVials; i++) {
        if (spectro_lbls_[i]) {
            float val = msg->spectro_absorbance[i];
            spectro_lbls_[i]->setText(
                QString("%1 AU").arg(static_cast<double>(val), 0, 'f', 3));
            spectro_lbls_[i]->setStyleSheet(
                msg->spectro_ready ? kSensorGreen : kSensorCell);
        }
    }
    if (msg->spectro_ready) {
        setCheck(3, 1, true);
        bool all6 = true;
        for (float v : msg->spectro_absorbance)
            if (v <= 0.0f) { all6 = false; break; }
        if (all6) setCheck(3, 2, true);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Live checklist + auto-advance
// ─────────────────────────────────────────────────────────────────────────────

QPushButton* SciencePipelineModule::makeCheck(int step, int idx, const QString& text)
{
    auto* b = new QPushButton("  ○  " + text);
    b->setCheckable(true);
    b->setFont(QFont("monospace", theme::FontSize));
    b->setStyleSheet(kCheckOff);
    b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    checks_[step][idx] = b;
    QObject::connect(b, &QPushButton::toggled, [this, b, text](bool on) {
        b->setText((on ? "  ✓  " : "  ○  ") + text);
        b->setStyleSheet(on ? kCheckOn : kCheckOff);
        evalAdvance();
    });
    return b;
}

void SciencePipelineModule::setCheck(int step, int idx, bool on)
{
    if (checks_[step][idx] && checks_[step][idx]->isChecked() != on)
        checks_[step][idx]->setChecked(on);
}

void SciencePipelineModule::setVial(int idx, bool filled)
{
    if (!vial_btns_[idx] || vials_filled_[idx] == filled) return;
    vials_filled_[idx] = filled;
    vial_btns_[idx]->setText(
        QString(filled ? "✓ Vial %1" : "○ Vial %1").arg(idx + 1));
    vial_btns_[idx]->setStyleSheet(filled ? kVialOn : kVialOff);

    int n = 0;
    for (bool f : vials_filled_) n += f ? 1 : 0;
    if (vials_count_lbl_)
        vials_count_lbl_->setText(QString("%1 / 6 vials filled").arg(n));
    setCheck(2, 2, n == 6);
}

bool SciencePipelineModule::stepSatisfied(int s) const
{
    for (int i = 0; i < 3; i++)
        if (!checks_[s][i] || !checks_[s][i]->isChecked()) return false;
    return true;
}

void SciencePipelineModule::evalAdvance()
{
    while (current_step_ <= 3 && !steps_done_[current_step_] &&
           stepSatisfied(current_step_))
        advanceStep();
}

// ─────────────────────────────────────────────────────────────────────────────

PLUGINLIB_EXPORT_CLASS(SciencePipelineModule, rover_hmi_core::GuiModule)
