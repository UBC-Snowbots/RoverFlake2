// motor_config_module.h  —  "Motor Params"
//
// Flash-truth parameter panel.  One motor at a time (selector on top);
// rows are registers with three columns:
//   Default — compile-time motor_config.h value (read-only)
//   Actual  — value read back from the controller by the driver via
//             "conf get" (MoteusArmStatus.config[]); '?' until read
//   New     — edit box; Enter opens a confirm dialog:
//             [Apply (RAM)] [Apply & Write Flash] [Cancel]
//
// Persistence: controller flash is the persistence (write_flash=true adds
// "conf write").  Apply & Write Flash also records the value in the
// repo-tracked config/arm_params.ini (same path resolution as config/layout)
// so intentional changes are reviewable; the Actual column highlights drift
// against that file.  There is no hidden re-apply at startup.
//
// Calibration buttons unchanged (moteus_tool hall calibration flow).

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
#include "rover_msgs/msg/moteus_calibration_request.hpp"
#include "rover_msgs/msg/moteus_calibration_status.hpp"
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
            { "Enter (New col)", "Confirm dialog: RAM / flash / cancel"   },
            { "Refresh",         "Re-read all values from controllers"    },
            { "Calibrate",       "Run Hall calibration for this motor"    },
            { "Flash Defaults",  "Write motor_config.h defaults to all connected motors" },
        };
    }

    static constexpr int NUM_REGS = 12;   // 11 editable + gear (read-only)

private:
    void onFeedback(const rover_msgs::msg::MoteusArmStatus::SharedPtr msg);
    void refreshDisplay();                // repaint Default/Actual for selected motor
    void confirmAndApply(int reg_row);    // dialog + publish
    void flashAllDefaults();              // confirm + queue defaults for connected motors
    void pumpFlashQueue();                // paced sender: one register per tick

    QComboBox*   motor_sel_   = nullptr;
    QLabel*      defaults_[NUM_REGS] = {};
    QLabel*      actuals_[NUM_REGS]  = {};
    QLineEdit*   edits_[NUM_REGS]    = {};
    QPushButton* calib_btn_   = nullptr;
    QPushButton* calib_all_btn_ = nullptr;
    QPushButton* flash_all_btn_ = nullptr;
    QLabel*      status_      = nullptr;

    // Paced write queue for Flash Defaults (QoS depth is 1 — bursts drop).
    std::vector<rover_msgs::msg::MoteusConfigUpdate> flash_queue_;
    QTimer* flash_timer_ = nullptr;
    int     flash_total_ = 0;

    rover_msgs::msg::MoteusArmStatus latest_;   // last feedback (config + connected)
    bool have_msg_ = false;

    rclcpp::Subscription<rover_msgs::msg::MoteusArmStatus>::SharedPtr        sub_;
    rclcpp::Publisher<rover_msgs::msg::MoteusConfigUpdate>::SharedPtr        pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr                       refresh_pub_;
    rclcpp::Subscription<rover_msgs::msg::MoteusCalibrationStatus>::SharedPtr calib_status_sub_;
    rclcpp::Publisher<rover_msgs::msg::MoteusCalibrationRequest>::SharedPtr  calib_pub_;
};
