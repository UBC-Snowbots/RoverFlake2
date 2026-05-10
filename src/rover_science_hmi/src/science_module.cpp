#include "science_module.h"

#include <QScrollArea>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFrame>
#include <QTimer>
#include <QSizePolicy>

#include <pluginlib/class_list_macros.hpp>
#include <rover_hmi_core/catppuccin.h>

// ─────────────────────────────────────────────────────────────────────────────
// Style helpers
// ─────────────────────────────────────────────────────────────────────────────

static QLabel* makeSectionLabel(const QString& title, QWidget* parent = nullptr)
{
    auto* lbl = new QLabel(title.toUpper(), parent);
    lbl->setStyleSheet(
        "color:#555555; font-size:11px; font-weight:bold;"
        " letter-spacing:2px; padding-top:10px;");
    return lbl;
}

static QFrame* makeDivider(QWidget* parent = nullptr)
{
    auto* line = new QFrame(parent);
    line->setFrameShape(QFrame::HLine);
    line->setStyleSheet("color:#222222;");
    return line;
}

static const char* kActive   = "QPushButton{background:#003322;color:#00ff88;border:1px solid #00ff88;border-radius:6px;padding:8px 14px;font-weight:bold;}";
static const char* kInactive = "QPushButton{background:#0a0a0a;color:#555555;border:1px solid #333333;border-radius:6px;padding:8px 14px;}";
static const char* kWarning  = "QPushButton{background:#332200;color:#ffcc00;border:1px solid #ffcc00;border-radius:6px;padding:8px 14px;font-weight:bold;}";
static const char* kPumpFwd  = "QPushButton{background:#003322;color:#00ff88;border:1px solid #00ff88;border-radius:6px;padding:8px 14px;font-weight:bold;}";
static const char* kPumpRev  = "QPushButton{background:#332200;color:#ffcc00;border:1px solid #ffcc00;border-radius:6px;padding:8px 14px;font-weight:bold;}";
static const char* kPumpStop = "QPushButton{background:#111111;color:#ffffff;border:1px solid #ffffff;border-radius:6px;padding:8px 14px;font-weight:bold;}";
static const char* kPumpDim  = "QPushButton{background:#0a0a0a;color:#444444;border:1px solid #222222;border-radius:6px;padding:8px 14px;}";
static const char* kSeqOn    = "QPushButton{background:#332600;color:#ffcc00;border:1px solid #ffcc00;border-radius:6px;padding:8px 14px;font-weight:bold;}";
static const char* kOSFOk    = "QPushButton{background:#003322;color:#00ff88;border:1px solid #00ff88;border-radius:6px;padding:8px 14px;font-weight:bold;}";
static const char* kOSFBad   = "QPushButton{background:#330011;color:#ff4466;border:1px solid #ff4466;border-radius:6px;padding:8px 14px;font-weight:bold;}";

// ─────────────────────────────────────────────────────────────────────────────
// createWidget
// ─────────────────────────────────────────────────────────────────────────────

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
    root->setContentsMargins(12, 8, 12, 12);
    root->setSpacing(4);

    // ── E-STOP ──────────────────────────────────────────────────────────────
    estop_btn_ = new QPushButton("  E-STOP  ");
    estop_btn_->setStyleSheet(
        "QPushButton{background:#ff4466;color:#000000;border:none;"
        "border-radius:8px;padding:14px;font-size:16px;font-weight:bold;}"
        "QPushButton:hover{background:#ff6680;}");
    estop_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    QObject::connect(estop_btn_, &QPushButton::clicked, [this]() { eStop(); });
    root->addWidget(estop_btn_);

    // ── SEQUENCES ───────────────────────────────────────────────────────────
    root->addWidget(makeSectionLabel("Sequences"));
    root->addWidget(makeDivider());

    auto* seqGrid = new QGridLayout();
    seqGrid->setSpacing(6);
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

    seq_status_lbl_ = new QLabel("Idle");
    seq_status_lbl_->setStyleSheet("color:#555555;font-size:12px;padding:2px 0;");
    root->addWidget(seq_status_lbl_);

    // ── VALVES ──────────────────────────────────────────────────────────────
    root->addWidget(makeSectionLabel("Solenoid Valves"));
    root->addWidget(makeDivider());

    auto* valveRow = new QHBoxLayout();
    valveRow->setSpacing(6);
    const char* svNames[] = { "SV1", "SV2", "SV3", "SV4" };
    for (int i = 0; i < 4; i++) {
        sv_btns_[i] = new QPushButton(svNames[i]);
        sv_btns_[i]->setStyleSheet(kInactive);
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
        valveRow->addWidget(sv_btns_[i]);
    }
    root->addLayout(valveRow);

    // ── PUMP ────────────────────────────────────────────────────────────────
    root->addWidget(makeSectionLabel("Pump P1"));
    root->addWidget(makeDivider());

    auto* pumpRow = new QHBoxLayout();
    pumpRow->setSpacing(6);
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

    // ── CAROUSEL ────────────────────────────────────────────────────────────
    root->addWidget(makeSectionLabel("Carousel"));
    root->addWidget(makeDivider());

    auto* carouselRow = new QHBoxLayout();
    carouselRow->setSpacing(6);
    auto* prevBtn  = new QPushButton("◀  Prev");
    carousel_lbl_  = new QLabel("  0  ");
    auto* nextBtn  = new QPushButton("Next  ▶");
    auto* resetBtn = new QPushButton("Reset");

    carousel_lbl_->setStyleSheet(
        "color:#ffffff;font-size:22px;font-weight:bold;"
        "background:#0a0a0a;border:1px solid #333333;"
        "border-radius:6px;padding:6px 16px;");
    carousel_lbl_->setAlignment(Qt::AlignCenter);

    for (auto* b : { prevBtn, nextBtn, resetBtn })
        b->setStyleSheet(kInactive);

    QObject::connect(prevBtn, &QPushButton::clicked, [this]() {
        carousel_idx_ = (carousel_idx_ == 0) ? 15 : carousel_idx_ - 1;
        state_.carouseldir   = -1;
        state_.carouselindex = static_cast<int16_t>(carousel_idx_);
        carousel_lbl_->setText(QString("  %1  ").arg(carousel_idx_));
        publish();
        QTimer::singleShot(200, carousel_lbl_, [this]() {
            state_.carouseldir = 0; publish();
        });
    });
    QObject::connect(nextBtn, &QPushButton::clicked, [this]() {
        carousel_idx_ = (carousel_idx_ == 15) ? 0 : carousel_idx_ + 1;
        state_.carouseldir   = 1;
        state_.carouselindex = static_cast<int16_t>(carousel_idx_);
        carousel_lbl_->setText(QString("  %1  ").arg(carousel_idx_));
        publish();
        QTimer::singleShot(200, carousel_lbl_, [this]() {
            state_.carouseldir = 0; publish();
        });
    });
    QObject::connect(resetBtn, &QPushButton::clicked, [this]() {
        carousel_idx_        = 0;
        state_.carouseldir   = 0;
        state_.carouselindex = 0;
        carousel_lbl_->setText("  0  ");
        publish();
    });

    carouselRow->addWidget(prevBtn);
    carouselRow->addWidget(carousel_lbl_, 1);
    carouselRow->addWidget(nextBtn);
    carouselRow->addWidget(resetBtn);
    root->addLayout(carouselRow);

    // ── DISPENSING ──────────────────────────────────────────────────────────
    root->addWidget(makeSectionLabel("Dispensing"));
    root->addWidget(makeDivider());

    auto* dispRow = new QHBoxLayout();
    dispRow->setSpacing(6);
    large_btn_ = new QPushButton("LARGE");
    small_btn_ = new QPushButton("SMALL");
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
    osfRow->setSpacing(6);
    osf1_btn_ = new QPushButton("OSF 1  OK");
    osf2_btn_ = new QPushButton("OSF 2  OK");
    for (auto* b : { osf1_btn_, osf2_btn_ }) {
        b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        osfRow->addWidget(b);
    }
    state_.osf1status = 1;
    state_.osf2status = 1;
    // updateOSFWarning() called after osf_warn_lbl_ is created below

    QObject::connect(osf1_btn_, &QPushButton::clicked, [this]() {
        setOSF(0, state_.osf1status == 0);
    });
    QObject::connect(osf2_btn_, &QPushButton::clicked, [this]() {
        setOSF(1, state_.osf2status == 0);
    });

    osf_warn_lbl_ = new QLabel("");
    osf_warn_lbl_->setStyleSheet("color:#ff4466;font-weight:bold;padding:2px 0;");
    root->addLayout(osfRow);
    root->addWidget(osf_warn_lbl_);

    // Now that osf_warn_lbl_ exists, safe to call
    updateOSFWarning();

    // ── UTILITIES ───────────────────────────────────────────────────────────
    root->addWidget(makeSectionLabel("Utilities"));
    root->addWidget(makeDivider());

    auto* utilGrid = new QGridLayout();
    utilGrid->setSpacing(6);
    spectro_btn_ = new QPushButton("Spectrometer");
    ag_btn_      = new QPushButton("Agitator Pwr");
    light1_btn_  = new QPushButton("Light 1");
    light2_btn_  = new QPushButton("Light 2");

    QPushButton* utilBtns[] = { spectro_btn_, ag_btn_, light1_btn_, light2_btn_ };
    for (int i = 0; i < 4; i++) {
        utilBtns[i]->setStyleSheet(kInactive);
        utilBtns[i]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        utilGrid->addWidget(utilBtns[i], i / 2, i % 2);
    }
    QObject::connect(spectro_btn_, &QPushButton::clicked, [this]() {
        state_.spectrostatus = (state_.spectrostatus == 1) ? 0 : 1;
        spectro_btn_->setStyleSheet(state_.spectrostatus ? kActive : kInactive);
        publish();
    });
    QObject::connect(ag_btn_, &QPushButton::clicked, [this]() {
        state_.agpowerstatus = (state_.agpowerstatus == 1) ? 0 : 1;
        ag_btn_->setStyleSheet(state_.agpowerstatus ? kActive : kInactive);
        publish();
    });
    QObject::connect(light1_btn_, &QPushButton::clicked, [this]() {
        state_.light1status = (state_.light1status == 1) ? 0 : 1;
        light1_btn_->setStyleSheet(state_.light1status ? kActive : kInactive);
        publish();
    });
    QObject::connect(light2_btn_, &QPushButton::clicked, [this]() {
        state_.light2status = (state_.light2status == 1) ? 0 : 1;
        light2_btn_->setStyleSheet(state_.light2status ? kActive : kInactive);
        publish();
    });

    root->addLayout(utilGrid);
    root->addStretch();

    publish();
    scroll->setWidget(container);
    return scroll;
}

// ─────────────────────────────────────────────────────────────────────────────
// setNode
// ─────────────────────────────────────────────────────────────────────────────

void ScienceModule::setNode(rclcpp::Node::SharedPtr node)
{
    node_ = node;
    pub_ = node_->create_publisher<rover_msgs::msg::ScienceModule>(
        "/science/command",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
}

// ─────────────────────────────────────────────────────────────────────────────
// publish
// ─────────────────────────────────────────────────────────────────────────────

void ScienceModule::publish()
{
    if (pub_) pub_->publish(state_);
}

// ─────────────────────────────────────────────────────────────────────────────
// E-STOP
// ─────────────────────────────────────────────────────────────────────────────

void ScienceModule::eStop()
{
    int saved_carousel = carousel_idx_;
    state_ = rover_msgs::msg::ScienceModule{};
    state_.carouselindex = static_cast<int16_t>(saved_carousel);
    state_.osf1status = 1;
    state_.osf2status = 1;
    is_purging_ = false;

    for (int i = 0; i < 4; i++) updateValveBtn(i);
    updatePumpBtns();
    updateSeqBtns();
    updateDispenserBtns();
    updateOSFWarning();
    spectro_btn_->setStyleSheet(kInactive);
    ag_btn_     ->setStyleSheet(kInactive);
    light1_btn_ ->setStyleSheet(kInactive);
    light2_btn_ ->setStyleSheet(kInactive);

    seq_status_lbl_->setText("E-STOP — all outputs zeroed");
    seq_status_lbl_->setStyleSheet("color:#ff4466;font-size:12px;font-weight:bold;");
    publish();

    QTimer::singleShot(2000, seq_status_lbl_, [this]() {
        seq_status_lbl_->setText("Idle");
        seq_status_lbl_->setStyleSheet("color:#555555;font-size:12px;");
    });
}

// ─────────────────────────────────────────────────────────────────────────────
// resetState — safe idle after a sequence completes
// ─────────────────────────────────────────────────────────────────────────────

void ScienceModule::resetState()
{
    state_.sequenceselection = 0;
    state_.sv1status = 0; state_.sv2status = 0;
    state_.sv3status = 0; state_.sv4status = 0;
    state_.p1status  = 0;
    is_purging_ = false;

    for (int i = 0; i < 4; i++) updateValveBtn(i);
    updatePumpBtns();
    updateSeqBtns();
    publish();
}

// ─────────────────────────────────────────────────────────────────────────────
// Valves
// ─────────────────────────────────────────────────────────────────────────────

void ScienceModule::setValve(int idx, bool on)
{
    int16_t* fields[] = {
        &state_.sv1status, &state_.sv2status,
        &state_.sv3status, &state_.sv4status
    };
    *fields[idx] = on ? 1 : 0;
    updateValveBtn(idx);
    publish();
}

void ScienceModule::updateValveBtn(int idx)
{
    if (!sv_btns_[idx]) return;
    const int16_t* fields[] = {
        &state_.sv1status, &state_.sv2status,
        &state_.sv3status, &state_.sv4status
    };
    const char* names[] = { "SV1", "SV2", "SV3", "SV4" };
    bool on = (*fields[idx] == 1);
    sv_btns_[idx]->setText(on ? QString("%1  ●").arg(names[idx])
                              : QString("%1  ○").arg(names[idx]));
    sv_btns_[idx]->setStyleSheet(on ? kActive : kInactive);
}

// ─────────────────────────────────────────────────────────────────────────────
// Pump
// ─────────────────────────────────────────────────────────────────────────────

void ScienceModule::setPump(int status)
{
    state_.p1status = static_cast<int16_t>(status);
    updatePumpBtns();
    publish();
}

void ScienceModule::updatePumpBtns()
{
    if (!pump_rev_btn_ || !pump_stop_btn_ || !pump_fwd_btn_) return;
    pump_rev_btn_ ->setStyleSheet(state_.p1status == 1 ? kPumpRev  : kPumpDim);
    pump_stop_btn_->setStyleSheet(state_.p1status == 0 ? kPumpStop : kPumpDim);
    pump_fwd_btn_ ->setStyleSheet(state_.p1status == 2 ? kPumpFwd  : kPumpDim);
}

// ─────────────────────────────────────────────────────────────────────────────
// OSF
// ─────────────────────────────────────────────────────────────────────────────

void ScienceModule::setOSF(int idx, bool unblocked)
{
    int16_t* fields[] = { &state_.osf1status, &state_.osf2status };
    *fields[idx] = unblocked ? 1 : 0;
    if (!unblocked && !is_purging_) eStop();
    updateOSFWarning();
    publish();
}

void ScienceModule::updateOSFWarning()
{
    if (!osf1_btn_ || !osf2_btn_ || !osf_warn_lbl_) return;
    bool ok1 = (state_.osf1status == 1);
    bool ok2 = (state_.osf2status == 1);
    osf1_btn_->setText(ok1 ? "OSF 1  OK" : "OSF 1  BLOCKED");
    osf2_btn_->setText(ok2 ? "OSF 2  OK" : "OSF 2  BLOCKED");
    osf1_btn_->setStyleSheet(ok1 ? kOSFOk : kOSFBad);
    osf2_btn_->setStyleSheet(ok2 ? kOSFOk : kOSFBad);
    osf_warn_lbl_->setText((!ok1 || !ok2) ? "⚠  OSF blocked — auto E-STOP triggered" : "");
}

// ─────────────────────────────────────────────────────────────────────────────
// Sequences
//
//  Timing matches the original GTK implementation:
//    Rinse:    2000 → step0 → 3000 → step1 → 4000 → done
//    Agitator: 2000 → step0 → 1000 → step1 → 5000 → done
//    Process:  2000 → step0 → 4000 → done
//    Purge:    2000 → step0 → 3000 → step1 → 6000 → done
// ─────────────────────────────────────────────────────────────────────────────

void ScienceModule::startSequence(int seq)
{
    cancelSequence();
    state_.sequenceselection = static_cast<int16_t>(seq);
    seq_step_ = 0;
    is_purging_ = (seq == 4);
    updateSeqBtns();

    const char* names[] = { "", "Rinse", "Agitator", "Process", "Purge" };
    seq_status_lbl_->setStyleSheet("color:#ffcc00;font-size:12px;font-weight:bold;");
    seq_status_lbl_->setText(QString("%1 — starting…").arg(names[seq]));
    publish();

    scheduleStep(2000, seq, [this, seq]() { advanceSequence(seq); });
}

void ScienceModule::cancelSequence()
{
    if (state_.sequenceselection == 0) return;
    state_.sequenceselection = 0;
    is_purging_ = false;
    updateSeqBtns();
    seq_status_lbl_->setText("Cancelled");
    seq_status_lbl_->setStyleSheet("color:#555555;font-size:12px;");
    publish();
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
                    resetState();
                    seq_status_lbl_->setText("Rinse — complete");
                    seq_status_lbl_->setStyleSheet("color:#00ff88;font-size:12px;");
                    QTimer::singleShot(2000, seq_status_lbl_, [this]() {
                        seq_status_lbl_->setText("Idle");
                        seq_status_lbl_->setStyleSheet("color:#555555;font-size:12px;");
                    });
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
                    resetState();
                    seq_status_lbl_->setText("Agitator — complete");
                    seq_status_lbl_->setStyleSheet("color:#00ff88;font-size:12px;");
                    QTimer::singleShot(2000, seq_status_lbl_, [this]() {
                        seq_status_lbl_->setText("Idle");
                        seq_status_lbl_->setStyleSheet("color:#555555;font-size:12px;");
                    });
                    return;
            }
            break;

        case 3: // Process
            switch (seq_step_) {
                case 0:
                    carousel_idx_ = 0;
                    state_.carouselindex = 0; state_.carouseldir = 0;
                    carousel_lbl_->setText("  0  ");
                    state_.sv1status = 1; state_.p1status = 2;
                    updateValveBtn(0); updatePumpBtns();
                    seq_status_lbl_->setText("Process — step 1/2: pump fwd");
                    next_delay_ms = 4000;
                    break;
                case 1:
                    resetState();
                    seq_status_lbl_->setText("Process — complete");
                    seq_status_lbl_->setStyleSheet("color:#00ff88;font-size:12px;");
                    QTimer::singleShot(2000, seq_status_lbl_, [this]() {
                        seq_status_lbl_->setText("Idle");
                        seq_status_lbl_->setStyleSheet("color:#555555;font-size:12px;");
                    });
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
                    resetState();
                    seq_status_lbl_->setText("Purge — complete");
                    seq_status_lbl_->setStyleSheet("color:#00ff88;font-size:12px;");
                    QTimer::singleShot(2000, seq_status_lbl_, [this]() {
                        seq_status_lbl_->setText("Idle");
                        seq_status_lbl_->setStyleSheet("color:#555555;font-size:12px;");
                    });
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
            state_.sequenceselection == (i + 1) ? kSeqOn : kInactive);
}

// ─────────────────────────────────────────────────────────────────────────────
// Dispensers
// ─────────────────────────────────────────────────────────────────────────────

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

// ─────────────────────────────────────────────────────────────────────────────

PLUGINLIB_EXPORT_CLASS(ScienceModule, rover_hmi_core::GuiModule)
