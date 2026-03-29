#pragma once

#include <rover_hmi_core/gui_module.h>
#include <QLabel>
#include <QPushButton>
#include <array>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/science_module.hpp"

class ScienceModule : public rover_hmi_core::GuiModule
{
public:
    std::string name()       const override { return "Science"; }
    std::string layoutHint() const override { return "main"; }
    std::string sectionName() const override { return "Science"; }
    QWidget*    createWidget(QWidget* parent) override;
    void        setNode(rclcpp::Node::SharedPtr node) override;

private:
    // ── Published state ────────────────────────────────────────────────────
    rover_msgs::msg::ScienceModule state_{};

    // ── Runtime flags ──────────────────────────────────────────────────────
    bool is_purging_   = false;   // purge sequence exempts OSF auto-stop
    int  carousel_idx_ = 0;
    int  seq_step_     = 0;       // current step within active sequence
    int  disp_step_    = 0;       // current step within dispenser actuation

    // ── UI handles ─────────────────────────────────────────────────────────
    QPushButton* estop_btn_      = nullptr;

    std::array<QPushButton*, 4> sv_btns_{};   // SV1–SV4

    QPushButton* pump_rev_btn_   = nullptr;
    QPushButton* pump_stop_btn_  = nullptr;
    QPushButton* pump_fwd_btn_   = nullptr;

    QLabel*      carousel_lbl_   = nullptr;

    QPushButton* large_btn_      = nullptr;
    QPushButton* small_btn_      = nullptr;

    QPushButton* osf1_btn_       = nullptr;
    QPushButton* osf2_btn_       = nullptr;
    QLabel*      osf_warn_lbl_   = nullptr;

    QPushButton* spectro_btn_    = nullptr;
    QPushButton* ag_btn_         = nullptr;
    QPushButton* light1_btn_     = nullptr;
    QPushButton* light2_btn_     = nullptr;

    std::array<QPushButton*, 4> seq_btns_{};  // Rinse, Agitator, Process, Purge
    QLabel*      seq_status_lbl_ = nullptr;

    // ── ROS ────────────────────────────────────────────────────────────────
    rclcpp::Node::SharedPtr    node_;
    rclcpp::Publisher<rover_msgs::msg::ScienceModule>::SharedPtr pub_;

    // ── Helpers ────────────────────────────────────────────────────────────
    void publish();
    void eStop();
    void resetState();

    void setValve(int idx, bool on);          // idx: 0–3 → SV1–SV4
    void setPump(int status);                 // 0=stop, 1=rev, 2=fwd
    void setOSF(int idx, bool unblocked);     // idx: 0 or 1
    void updateOSFWarning();

    // Sequences (sequenceselection 1=rinse 2=agitator 3=process 4=purge; 0=idle)
    void startSequence(int seq);
    void cancelSequence();
    void advanceSequence(int seq);
    void scheduleStep(int delay_ms, int owning_seq, std::function<void()> fn);

    // Dispenser actuations (not a sequenceselection value — independent)
    void startDispenser(bool large);
    void advanceDispenser(bool large);

    // Visual helpers
    void updateValveBtn(int idx);
    void updatePumpBtns();
    void updateSeqBtns();
    void updateDispenserBtns();
};
