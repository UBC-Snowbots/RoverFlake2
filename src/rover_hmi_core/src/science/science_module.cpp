#include "science_module.h"

#include <QScrollArea>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFrame>
#include <QTimer>
#include <QSizePolicy>
#include <utility>

#include <pluginlib/class_list_macros.hpp>
#include <rover_hmi_core/catppuccin.h>

// Style helpers ──────────────────────────────────────────────────────────────

static QString btnStyle(const char* bg, const char* fg, const char* border, bool bold = true)
{
    return QString(
        "QPushButton{background:%1;color:%2;border:1px solid %3;"
        "border-radius:5px;padding:8px 10px;font-size:%5px;%4}")
        .arg(bg, fg, border, bold ? "font-weight:bold;" : "").arg(theme::FontSize);
}

static const QString kActive   = btnStyle("#003322", theme::Green, theme::Green);
static const QString kInactive = btnStyle(theme::BgPanel, "#999999", theme::BorderDim, false);
static const QString kWarning  = btnStyle("#332200", theme::Yellow, theme::Yellow);
static const QString kStop     = btnStyle(theme::HeaderBg, theme::Text, theme::Text);
static const QString kDim      = btnStyle(theme::BgPanel, "#555555", "#222222", false);
static const QString kBad      = btnStyle("#330011", theme::Red, theme::Red);

static QLabel* makeSectionLabel(const QString& title, QWidget* parent = nullptr)
{
    auto* lbl = new QLabel(title.toUpper(), parent);
    lbl->setStyleSheet(QString(
        "color:#777777;font-size:%1px;font-weight:bold;"
        "letter-spacing:2px;padding-top:6px;").arg(theme::FontSizeSm));
    return lbl;
}

static QFrame* makeDivider(QWidget* parent = nullptr)
{
    auto* line = new QFrame(parent);
    line->setFrameShape(QFrame::HLine);
    line->setStyleSheet("color:#222222;");
    return line;
}

// createWidget ───────────────────────────────────────────────────────────────

QWidget* ScienceModule::createWidget(QWidget* parent)
{
    auto* scroll = new QScrollArea(parent);
    scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setStyleSheet(
        "QScrollArea{border:none;background:#000000;}"
        "QScrollBar:vertical{width:6px;background:#0a0a0a;}"
        "QScrollBar::handle:vertical{background:#333333;border-radius:3px;}");

    auto* container = new QWidget();
    container->setStyleSheet("background:#000000;");
    auto* root = new QVBoxLayout(container);
    root->setContentsMargins(8, 6, 8, 8);
    root->setSpacing(3);

    // ── Drilling ────────────────────────────────────────────────────────────
    root->addWidget(makeSectionLabel("Drilling"));
    root->addWidget(makeDivider());

    auto* drillRow = new QHBoxLayout();
    drillRow->setSpacing(4);
    drill_btn_ = new QPushButton("Drill Motor");
    vac_btn_   = new QPushButton("Vacuum");
    for (auto* b : { drill_btn_, vac_btn_ }) {
        b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        drillRow->addWidget(b);
    }
    QObject::connect(drill_btn_, &QPushButton::clicked, [this]() {
        state_.drillmotorstatus = (state_.drillmotorstatus == 1) ? 0 : 1;
        updateDrillBtns();
        publish();
    });
    QObject::connect(vac_btn_, &QPushButton::clicked, [this]() {
        state_.vacuumstatus = (state_.vacuumstatus == 1) ? 0 : 1;
        updateDrillBtns();
        publish();
    });
    root->addLayout(drillRow);

    auto* stepperRow = new QHBoxLayout();
    stepperRow->setSpacing(4);
    step_lower_btn_ = new QPushButton("Lower ▼");
    step_hold_btn_  = new QPushButton("Hold");
    step_raise_btn_ = new QPushButton("Raise ▲");
    const std::pair<QPushButton*, int> dirs[] = {
        { step_lower_btn_, 1 }, { step_hold_btn_, 0 }, { step_raise_btn_, -1 } };
    for (auto [b, dir] : dirs) {
        b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        QObject::connect(b, &QPushButton::clicked, [this, dir = dir]() {
            state_.drillstepperdir = static_cast<int8_t>(dir);
            updateDrillBtns();
            publish();
        });
        stepperRow->addWidget(b);
    }
    root->addLayout(stepperRow);
    updateDrillBtns();

    // ── Sequences ───────────────────────────────────────────────────────────
    root->addWidget(makeSectionLabel("Sequences"));
    root->addWidget(makeDivider());

    auto* seqGrid = new QGridLayout();
    seqGrid->setSpacing(4);
    const char* seqNames[] = { "Rinse", "Agitator", "Process", "Purge" };
    for (int i = 0; i < 4; i++) {
        seq_btns_[i] = new QPushButton(seqNames[i]);
        seq_btns_[i]->setStyleSheet(kInactive);
        seq_btns_[i]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        int seq = i + 1;
        QObject::connect(seq_btns_[i], &QPushButton::clicked, [this, seq]() {
            if (state_.sequenceselection == seq) cancelSequence();
            else startSequence(seq);
        });
        seqGrid->addWidget(seq_btns_[i], i / 2, i % 2);
    }
    root->addLayout(seqGrid);

    seq_status_lbl_ = new QLabel();
    root->addWidget(seq_status_lbl_);
    setStatus("Idle", "#777777");

    // ── Valves ──────────────────────────────────────────────────────────────
    root->addWidget(makeSectionLabel("Solenoid Valves"));
    root->addWidget(makeDivider());

    auto* valveRow = new QHBoxLayout();
    valveRow->setSpacing(4);
    for (int i = 0; i < 4; i++) {
        sv_btns_[i] = new QPushButton();
        sv_btns_[i]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        QObject::connect(sv_btns_[i], &QPushButton::clicked, [this, i]() {
            int16_t* fields[] = {
                &state_.sv1status, &state_.sv2status,
                &state_.sv3status, &state_.sv4status
            };
            *fields[i] = (*fields[i] == 1) ? 0 : 1;
            updateValveBtn(i);
            publish();
        });
        updateValveBtn(i);
        valveRow->addWidget(sv_btns_[i]);
    }
    root->addLayout(valveRow);

    // ── Pump ────────────────────────────────────────────────────────────────
    root->addWidget(makeSectionLabel("Pump P1"));
    root->addWidget(makeDivider());

    auto* pumpRow = new QHBoxLayout();
    pumpRow->setSpacing(4);
    pump_rev_btn_  = new QPushButton("← REV");
    pump_stop_btn_ = new QPushButton("STOP");
    pump_fwd_btn_  = new QPushButton("FWD →");
    for (auto* b : { pump_rev_btn_, pump_stop_btn_, pump_fwd_btn_ }) {
        b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        pumpRow->addWidget(b);
    }
    QObject::connect(pump_rev_btn_,  &QPushButton::clicked, [this]() { setPump(1); });
    QObject::connect(pump_stop_btn_, &QPushButton::clicked, [this]() { setPump(0); });
    QObject::connect(pump_fwd_btn_,  &QPushButton::clicked, [this]() { setPump(2); });
    root->addLayout(pumpRow);
    updatePumpBtns();

    // ── Carousel ────────────────────────────────────────────────────────────
    root->addWidget(makeSectionLabel("Carousel"));
    root->addWidget(makeDivider());

    auto* carouselRow = new QHBoxLayout();
    carouselRow->setSpacing(4);
    auto* prevBtn  = new QPushButton("◀ Prev");
    carousel_lbl_  = new QLabel("0");
    auto* nextBtn  = new QPushButton("Next ▶");
    auto* resetBtn = new QPushButton("Reset");

    carousel_lbl_->setStyleSheet(QString(
        "color:#ffffff;font-size:%1px;font-weight:bold;"
        "background:#0a0a0a;border:1px solid #333333;"
        "border-radius:5px;padding:3px 12px;").arg(theme::FontSizeLg));
    carousel_lbl_->setAlignment(Qt::AlignCenter);

    auto bump = [this](int dir) {
        carousel_idx_ = (carousel_idx_ + dir + 16) % 16;
        state_.carouseldir   = static_cast<int16_t>(dir);
        state_.carouselindex = static_cast<int16_t>(carousel_idx_);
        carousel_lbl_->setText(QString::number(carousel_idx_));
        publish();
        QTimer::singleShot(200, carousel_lbl_, [this]() {
            state_.carouseldir = 0; publish();
        });
    };
    QObject::connect(prevBtn, &QPushButton::clicked, [bump]() { bump(-1); });
    QObject::connect(nextBtn, &QPushButton::clicked, [bump]() { bump(1); });
    QObject::connect(resetBtn, &QPushButton::clicked, [this]() {
        carousel_idx_        = 0;
        state_.carouseldir   = 0;
        state_.carouselindex = 0;
        carousel_lbl_->setText("0");
        publish();
    });

    for (auto* b : { prevBtn, nextBtn, resetBtn })
        b->setStyleSheet(kInactive);
    carouselRow->addWidget(prevBtn);
    carouselRow->addWidget(carousel_lbl_, 1);
    carouselRow->addWidget(nextBtn);
    carouselRow->addWidget(resetBtn);
    root->addLayout(carouselRow);

    // ── Dispensing ──────────────────────────────────────────────────────────
    root->addWidget(makeSectionLabel("Dispensing"));
    root->addWidget(makeDivider());

    auto* dispRow = new QHBoxLayout();
    dispRow->setSpacing(4);
    large_btn_ = new QPushButton("Large");
    small_btn_ = new QPushButton("Small");
    for (auto* b : { large_btn_, small_btn_ }) {
        b->setStyleSheet(kInactive);
        b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        dispRow->addWidget(b);
    }
    QObject::connect(large_btn_, &QPushButton::clicked, [this]() {
        if (state_.largestatus == 0) startDispenser(true);
    });
    QObject::connect(small_btn_, &QPushButton::clicked, [this]() {
        if (state_.smallstatus == 0) startDispenser(false);
    });
    root->addLayout(dispRow);

    // ── OSF ─────────────────────────────────────────────────────────────────
    root->addWidget(makeSectionLabel("Optical Sensors (OSF)"));
    root->addWidget(makeDivider());

    auto* osfRow = new QHBoxLayout();
    osfRow->setSpacing(4);
    osf1_btn_ = new QPushButton();
    osf2_btn_ = new QPushButton();
    for (auto* b : { osf1_btn_, osf2_btn_ }) {
        b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        osfRow->addWidget(b);
    }
    state_.osf1status = 1;
    state_.osf2status = 1;
    QObject::connect(osf1_btn_, &QPushButton::clicked, [this]() {
        setOSF(0, state_.osf1status == 0);
    });
    QObject::connect(osf2_btn_, &QPushButton::clicked, [this]() {
        setOSF(1, state_.osf2status == 0);
    });

    osf_warn_lbl_ = new QLabel();
    osf_warn_lbl_->setStyleSheet(QString(
        "color:#ff4466;font-size:%1px;font-weight:bold;padding:2px 0;")
        .arg(theme::FontSizeSm));
    root->addLayout(osfRow);
    root->addWidget(osf_warn_lbl_);
    updateOSFWarning();

    // ── Utilities ───────────────────────────────────────────────────────────
    root->addWidget(makeSectionLabel("Utilities"));
    root->addWidget(makeDivider());

    auto* utilGrid = new QGridLayout();
    utilGrid->setSpacing(4);
    struct UtilDef {
        QPushButton** btn;
        int16_t rover_msgs::msg::ScienceModule::* field;
        const char* label;
    };
    const UtilDef utils[] = {
        { &spectro_btn_, &rover_msgs::msg::ScienceModule::spectrostatus, "Spectrometer" },
        { &ag_btn_,      &rover_msgs::msg::ScienceModule::agpowerstatus, "Agitator Pwr" },
        { &light1_btn_,  &rover_msgs::msg::ScienceModule::light1status,  "Light 1" },
        { &light2_btn_,  &rover_msgs::msg::ScienceModule::light2status,  "Light 2" },
    };
    for (int i = 0; i < 4; i++) {
        auto* b = *utils[i].btn = new QPushButton(utils[i].label);
        auto f = utils[i].field;
        b->setStyleSheet(kInactive);
        b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        QObject::connect(b, &QPushButton::clicked, [this, b, f]() {
            state_.*f = (state_.*f == 1) ? 0 : 1;
            b->setStyleSheet(state_.*f ? kActive : kInactive);
            publish();
        });
        utilGrid->addWidget(b, i / 2, i % 2);
    }
    root->addLayout(utilGrid);
    root->addStretch();

    publish();
    scroll->setWidget(container);
    return scroll;
}

// ROS ────────────────────────────────────────────────────────────────────────

void ScienceModule::setNode(rclcpp::Node::SharedPtr node)
{
    node_ = node;
    pub_ = node_->create_publisher<rover_msgs::msg::ScienceModule>(
        "/science/command",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
}

void ScienceModule::publish()
{
    if (pub_) pub_->publish(state_);
}

// State helpers ──────────────────────────────────────────────────────────────

void ScienceModule::setStatus(const QString& text, const char* color, bool bold)
{
    seq_status_lbl_->setText(text);
    seq_status_lbl_->setStyleSheet(QString("color:%1;font-size:%2px;padding:2px 0;%3")
        .arg(color).arg(theme::FontSizeSm).arg(bold ? "font-weight:bold;" : ""));
}

// Safe idle after a sequence completes or is cancelled
void ScienceModule::resetState()
{
    state_.sequenceselection = 0;
    state_.sv1status = 0; state_.sv2status = 0;
    state_.sv3status = 0; state_.sv4status = 0;
    state_.p1status  = 0;

    for (int i = 0; i < 4; i++) updateValveBtn(i);
    updatePumpBtns();
    updateSeqBtns();
    publish();
}

void ScienceModule::updateValveBtn(int idx)
{
    if (!sv_btns_[idx]) return;
    const int16_t* fields[] = {
        &state_.sv1status, &state_.sv2status,
        &state_.sv3status, &state_.sv4status
    };
    bool on = (*fields[idx] == 1);
    sv_btns_[idx]->setText(QString("SV%1 %2").arg(idx + 1).arg(on ? "●" : "○"));
    sv_btns_[idx]->setStyleSheet(on ? kActive : kInactive);
}

void ScienceModule::setPump(int status)
{
    state_.p1status = static_cast<int16_t>(status);
    updatePumpBtns();
    publish();
}

void ScienceModule::updatePumpBtns()
{
    if (!pump_rev_btn_ || !pump_stop_btn_ || !pump_fwd_btn_) return;
    pump_rev_btn_ ->setStyleSheet(state_.p1status == 1 ? kWarning : kDim);
    pump_stop_btn_->setStyleSheet(state_.p1status == 0 ? kStop    : kDim);
    pump_fwd_btn_ ->setStyleSheet(state_.p1status == 2 ? kActive  : kDim);
}

void ScienceModule::updateDrillBtns()
{
    if (!drill_btn_ || !vac_btn_) return;
    drill_btn_->setStyleSheet(state_.drillmotorstatus ? kActive : kInactive);
    vac_btn_  ->setStyleSheet(state_.vacuumstatus     ? kActive : kInactive);
    step_lower_btn_->setStyleSheet(state_.drillstepperdir == 1  ? kWarning : kDim);
    step_hold_btn_ ->setStyleSheet(state_.drillstepperdir == 0  ? kStop    : kDim);
    step_raise_btn_->setStyleSheet(state_.drillstepperdir == -1 ? kActive  : kDim);
}

void ScienceModule::setOSF(int idx, bool unblocked)
{
    int16_t* fields[] = { &state_.osf1status, &state_.osf2status };
    *fields[idx] = unblocked ? 1 : 0;
    updateOSFWarning();
    publish();
}

void ScienceModule::updateOSFWarning()
{
    if (!osf1_btn_ || !osf2_btn_ || !osf_warn_lbl_) return;
    bool ok1 = (state_.osf1status == 1);
    bool ok2 = (state_.osf2status == 1);
    osf1_btn_->setText(ok1 ? "OSF 1 OK" : "OSF 1 BLOCKED");
    osf2_btn_->setText(ok2 ? "OSF 2 OK" : "OSF 2 BLOCKED");
    osf1_btn_->setStyleSheet(ok1 ? kActive : kBad);
    osf2_btn_->setStyleSheet(ok2 ? kActive : kBad);
    osf_warn_lbl_->setText((!ok1 || !ok2) ? "⚠ OSF blocked" : "");
}

// Sequences ──────────────────────────────────────────────────────────────────
//  Timing matches the original GTK implementation:
//    Rinse:    2000 → step0 → 3000 → step1 → 4000 → done
//    Agitator: 2000 → step0 → 1000 → step1 → 5000 → done
//    Process:  2000 → step0 → 4000 → done
//    Purge:    2000 → step0 → 3000 → step1 → 6000 → done

void ScienceModule::startSequence(int seq)
{
    cancelSequence();
    state_.sequenceselection = static_cast<int16_t>(seq);
    seq_step_ = 0;
    updateSeqBtns();

    const char* names[] = { "", "Rinse", "Agitator", "Process", "Purge" };
    setStatus(QString("%1 — starting…").arg(names[seq]), theme::Yellow, true);
    publish();

    scheduleStep(2000, seq, [this, seq]() { advanceSequence(seq); });
}

void ScienceModule::cancelSequence()
{
    if (state_.sequenceselection == 0) return;
    state_.sequenceselection = 0;
    updateSeqBtns();
    setStatus("Cancelled", "#777777");
    publish();
}

void ScienceModule::finishSequence(const QString& name)
{
    resetState();
    setStatus(name + " — complete", theme::Green);
    QTimer::singleShot(2000, seq_status_lbl_, [this]() {
        setStatus("Idle", "#777777");
    });
}

void ScienceModule::advanceSequence(int seq)
{
    if (state_.sequenceselection != seq) return;

    int next_delay_ms = 3000;

    switch (seq) {

        case 1: // Rinse
            switch (seq_step_) {
                case 0:
                    state_.sv2status = 1;
                    updateValveBtn(1);
                    seq_status_lbl_->setText("Rinse — step 1/3: SV2 open");
                    next_delay_ms = 3000;
                    break;
                case 1:
                    state_.sv2status = 0; state_.sv1status = 1;
                    state_.sv4status = 1; state_.p1status  = 2;
                    for (int i : {0,1,3}) updateValveBtn(i);
                    updatePumpBtns();
                    seq_status_lbl_->setText("Rinse — step 2/3: pump fwd");
                    next_delay_ms = 4000;
                    break;
                case 2:
                    finishSequence("Rinse");
                    return;
            }
            break;

        case 2: // Agitator
            switch (seq_step_) {
                case 0:
                    state_.sv2status = 1;
                    updateValveBtn(1);
                    seq_status_lbl_->setText("Agitator — step 1/3: SV2 open");
                    next_delay_ms = 1000;
                    break;
                case 1:
                    state_.sv2status = 0; state_.agpowerstatus = 1;
                    updateValveBtn(1);
                    ag_btn_->setStyleSheet(kActive);
                    seq_status_lbl_->setText("Agitator — step 2/3: agitating");
                    next_delay_ms = 5000;
                    break;
                case 2:
                    state_.agpowerstatus = 0;
                    ag_btn_->setStyleSheet(kInactive);
                    finishSequence("Agitator");
                    return;
            }
            break;

        case 3: // Process
            switch (seq_step_) {
                case 0:
                    carousel_idx_ = 0;
                    state_.carouselindex = 0; state_.carouseldir = 0;
                    carousel_lbl_->setText("0");
                    state_.sv1status = 1; state_.p1status = 2;
                    updateValveBtn(0); updatePumpBtns();
                    seq_status_lbl_->setText("Process — step 1/2: pump fwd");
                    next_delay_ms = 4000;
                    break;
                case 1:
                    finishSequence("Process");
                    return;
            }
            break;

        case 4: // Purge
            switch (seq_step_) {
                case 0:
                    state_.sv3status = 1; state_.p1status = 1;
                    updateValveBtn(2); updatePumpBtns();
                    seq_status_lbl_->setText("Purge — step 1/3: pump rev");
                    next_delay_ms = 3000;
                    break;
                case 1:
                    state_.sv3status = 1; state_.p1status = 2;
                    updatePumpBtns();
                    seq_status_lbl_->setText("Purge — step 2/3: pump fwd");
                    next_delay_ms = 6000;
                    break;
                case 2:
                    finishSequence("Purge");
                    return;
            }
            break;
    }

    publish();
    seq_step_++;
    scheduleStep(next_delay_ms, seq, [this, seq]() { advanceSequence(seq); });
}

void ScienceModule::scheduleStep(int delay_ms, int owning_seq,
                                 std::function<void()> fn)
{
    QTimer::singleShot(delay_ms, seq_status_lbl_, [this, owning_seq, fn]() {
        if (state_.sequenceselection != owning_seq) return;
        fn();
    });
}

void ScienceModule::updateSeqBtns()
{
    if (!seq_btns_[0]) return;
    for (int i = 0; i < 4; i++)
        seq_btns_[i]->setStyleSheet(
            state_.sequenceselection == (i + 1) ? kWarning : kInactive);
}

// Dispensers ─────────────────────────────────────────────────────────────────

void ScienceModule::startDispenser(bool large)
{
    disp_step_ = 0;
    if (large) state_.largestatus = 1;
    else       state_.smallstatus = 1;
    state_.sv4status = 0;
    updateValveBtn(3);
    updateDispenserBtns();
    publish();
    QTimer::singleShot(2000, large_btn_, [this, large]() {
        advanceDispenser(large);
    });
}

void ScienceModule::advanceDispenser(bool large)
{
    if (large  && state_.largestatus == 0) return;
    if (!large && state_.smallstatus == 0) return;

    disp_step_++;
    if (disp_step_ == 1) {
        state_.sv4status = 1;
        updateValveBtn(3);
        publish();
        QTimer::singleShot(large ? 1500 : 500, large_btn_, [this, large]() {
            advanceDispenser(large);
        });
    } else {
        state_.sv4status = 0;
        if (large) state_.largestatus = 0;
        else       state_.smallstatus = 0;
        disp_step_ = 0;
        updateValveBtn(3);
        updateDispenserBtns();
        publish();
    }
}

void ScienceModule::updateDispenserBtns()
{
    if (!large_btn_ || !small_btn_) return;
    large_btn_->setStyleSheet(state_.largestatus ? kWarning : kInactive);
    small_btn_->setStyleSheet(state_.smallstatus ? kWarning : kInactive);
}

PLUGINLIB_EXPORT_CLASS(ScienceModule, rover_hmi_core::GuiModule)
