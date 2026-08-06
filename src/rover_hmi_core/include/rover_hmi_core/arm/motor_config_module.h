// motor_config_module.h  —  "Motor Params"
//
// Flash-truth parameter panel.  One motor at a time (selector on top);
// rows are registers with three columns:
//   Default — compile-time motor_config.h value (read-only)
//   Actual  — value read back from the controller by the driver via
//             "conf get" (MoteusArmStatus.config[]); '?' until read
//   New     — edit box; Enter opens a confirm dialog: [Apply (RAM)] [Cancel]
//
// RAM-only by design: the HMI never writes flash and never calibrates —
// persistence and calibration are bench jobs (tview / moteus_tool). The
// repo-tracked config/arm_params.ini holds blessed values; the Actual
// column highlights drift against it. No hidden re-apply at startup.

#pragma once

#include <rover_hmi_core/gui_module.h>
#include <rover_arm_common/motor_addressing.h>

#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/moteus_arm_status.hpp"
#include "rover_msgs/msg/moteus_config_update.hpp"
#include "std_msgs/msg/empty.hpp"

class MotorConfigModule : public rover_hmi_core::GuiModule {
public:
    std::string name()        const override { return "Motor Params"; }
    std::string layoutHint()  const override { return "right"; }
    std::string sectionName() const override { return "Arm"; }

    QWidget* createWidget(QWidget* parent) override;
    void     setNode(rclcpp::Node::SharedPtr node) override;
    void     start() override {}
    void     stop() override {}

    std::vector<std::pair<std::string,std::string>> keybindings() const override {
        return {
            { "Motor selector",  "Choose which motor to view/edit"        },
            { "Enter (New col)", "Confirm dialog: apply to RAM / cancel"  },
            { "Refresh",         "Re-read all values from controllers"    },
        };
    }

    static constexpr int NUM_REGS = 12;   // 11 editable + gear (read-only)

    static QString fmtVal(float v);       // table formatting; '?' for NaN

private:
    friend struct MotorConfigTestAccess;  // test seam (test/motor_config_display_test.cpp)
    void onFeedback(const rover_msgs::msg::MoteusArmStatus::SharedPtr msg);
    void refreshDisplay();                // repaint Default/Actual for selected motor
    void confirmAndApply(int reg_row);    // dialog + publish

    QComboBox*   motor_sel_   = nullptr;
    QLabel*      defaults_[NUM_REGS] = {};
    QLabel*      actuals_[NUM_REGS]  = {};
    QLineEdit*   edits_[NUM_REGS]    = {};
    QLabel*      status_      = nullptr;

    rover_msgs::msg::MoteusArmStatus latest_;   // last feedback (config + connected)
    bool have_msg_ = false;

    rclcpp::Subscription<rover_msgs::msg::MoteusArmStatus>::SharedPtr        sub_;
    rclcpp::Publisher<rover_msgs::msg::MoteusConfigUpdate>::SharedPtr        pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr                       refresh_pub_;
};
