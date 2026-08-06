#pragma once

// =============================================================================
// Moteus Driver Node  (moteus_driver_node.h)
// =============================================================================
//
// ROS 2 node that runs the CAN-FD communication loop for the arm's moteus
// motor controllers.
//
// Related files — read these to understand the full system:
//
//   motor_addressing.h  — which motor ID maps to which physical joint,
//                         URDF joint names, direction signs, unit conversions
//
//   motor_config.h      — per-motor PID gains, current limits, position limits
//                         (the values pushed to firmware on startup)
//
//   moteus_protocol.h   — what goes in each CAN-FD data field, frame types,
//                         unit conventions, watchdog notes
//
//   arm_commands.h      — command codes (P/V/S) and the MotorCommand struct
//                         that carries commands from the ROS topic to the CAN loop
//
//   arm_telemetry.h     — MotorTelem struct: the per-motor state decoded from
//                         CAN reply frames each poll cycle
//
// TOPICS:
//   Subscribed:  /arm/command         (rover_msgs/ArmCommand)
//   Published:   /arm/moteus_feedback  (rover_msgs/MoteusArmStatus)
//                /joint_states         (sensor_msgs/JointState)  → RViz / Inverse Kin
//                /arm/config_log       (std_msgs/String)          → HMI log panel
// =============================================================================

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <array>
#include <map>
#include <bitset> // for printing out debug bitmasks

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/arm_command.hpp"
#include "rover_msgs/msg/moteus_arm_status.hpp"
#include "rover_msgs/msg/bldc_servo_status.hpp"
#include "rover_msgs/msg/bldc_servo_config.hpp"
#include "rover_msgs/msg/moteus_config_update.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// #include <rover_arm_common/motor_addressing.h>   // NUM_MOTORS, ARM_JOINTS, unit converters
#include <rover_arm_common/motor_config.h>       // MotorConfig, get_arm_configuration()
#include <rover_arm_common/arm_commands.h>       // CMD_*, MotorCommand
#include <rover_arm_common/arm_telemetry.h>      // MotorTelem
#include "moteus_protocol.h"                       // MoteusProtocol::makeFrame(), parseReply()

enum class AxisState
{
    INIT,
    REQUESTING_HOMING,
    HOMING,
    GOING_TO_PRESET_POSITION,
    RUNNING_OK,
    ERROR
};

struct Axis {
        int index; // Starts at 0
        float position = 0;
        bool limit_switch = 0;
        bool homed = false;
        AxisState state = AxisState::INIT;

        // CONFIG STUFF See motor_config.h -> Set during construction of this class
        float max_position_rev;
        float min_position_rev;
        float homing_speed_revps;
        int homing_direction; 
        // bool configured = false;
        // etc..
};

// Half assed debug printout flags - Uncomment to enable these printouts
// #define DEBUG_LIMIT_SWITCH_RAW_REPLY


class MoteusDriverNode : public rclcpp::Node {
public:
    MoteusDriverNode();

private:
    // Startup: push configuration to every motor via "conf set" DiagnosticCommand
    void configureMotors();
    void configureMotor(int motor_id, mot::Controller& ctrl);

    // "superloop" style. Blocking can transactions (100hz is slow for CANFD, so this should be okay)
    void run();

    // ROS subscription callback: fills pending_cmds_[] (mutex-protected)
    void commandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg);

    // ROS subscription callback: applies a single register update to one motor
    // via DiagnosticCommand("conf set ...") and updates configs_[] in memory.
    void configUpdateCallback(const rover_msgs::msg::MoteusConfigUpdate::SharedPtr msg);

    // Apply a (register, value) pair to the in-memory MotorConfig for motor idx.
    // Keeps configs_[] consistent so subsequent feedback messages reflect edits.
    void applyConfigToMemory(int idx, const std::string& reg, float val);

    // (Re-)create transport_ and controllers_ from fdcanusb detection.
    void reInitTransport();

    // Read the MotorConfig register set from one controller's flash/RAM via
    // "conf get".  Fills flash_cfg_[m]; published in config[] so the
    // HMI Motor Params panel shows actual values, not assumed ones.
    // Read-only bus traffic (SetQuery + conf get) — never writes config.
    // Returns true when a full register set was read.
    bool readFlashConfig(int m);

    // Service one pending flash-config read per interval, from run().  Reads
    // are staggered (one motor at a time, only motors currently replying) so a
    // full refresh never freezes the poll loop the way the old 7-motor
    // blocking sweep did.
    void serviceConfigReads();


    // Safety monitoring (called inside poll, only logs on state changes)
    void checkFaults();
    void checkAlerts();

    // Publish a string to /arm/config_log (shown in the HMI command log panel)
    void publishLog(const std::string& msg);

    void zero_position(uint8_t index);
    void set_position(uint8_t index, float position_revs);
    
    void home_axis(uint8_t index);

    Axis axes[NUM_AXES];

    // -------------------------------------------------------------------------
    // CAN transport + per-motor controllers
    // -------------------------------------------------------------------------
    std::shared_ptr<mot::Transport>              transport_;
    std::vector<std::shared_ptr<mot::Controller>> controllers_;  // index = motor_id - 1
    std::string can_device_path_;        // resolved /dev/ttyACM* we opened
    std::array<std::map<std::string, float>, NUM_MOTORS> flash_cfg_;  // conf get results
    bool flash_cfg_valid_[NUM_MOTORS] = {};
    // Staggered config reads (see serviceConfigReads): wanted = explicit
    // refresh request; motors with !flash_cfg_valid_ are re-read automatically
    // once they reply, so power-up order no longer matters.
    std::array<bool, NUM_MOTORS>    cfg_read_wanted_{};
    std::array<int64_t, NUM_MOTORS> cfg_read_last_ms_{};  // per-motor retry limiter
    std::array<int, NUM_MOTORS>     cfg_read_fails_{};    // give up after 3; Refresh resets
    int64_t cfg_read_gate_ms_ = 0;                        // global stagger gate
    int     cfg_read_next_    = 0;                        // round-robin cursor
    // In-flight sliced read: ONE register per poll tick so the 100 Hz loop
    // (and therefore telemetry) never stalls more than a single conf get.
    int         cfg_read_motor_ = -1;                     // -1 = idle
    size_t      cfg_read_reg_   = 0;
    std::map<std::string, float> cfg_read_accum_;

    // -------------------------------------------------------------------------
    // ROS interfaces
    // -------------------------------------------------------------------------
    rclcpp::TimerBase::SharedPtr                                      timer_;
    rclcpp::Publisher<rover_msgs::msg::MoteusArmStatus>::SharedPtr         feedback_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr             joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                    config_log_pub_;
    rclcpp::Publisher<rover_msgs::msg::ArmCommand>::SharedPtr           arm_feedback_pub;
    rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr           command_sub_;
    rclcpp::Subscription<rover_msgs::msg::MoteusConfigUpdate>::SharedPtr       config_update_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr                      config_refresh_sub_;

    // -------------------------------------------------------------------------
    // Configuration (loaded from motor_config.h at construction)
    // -------------------------------------------------------------------------
    std::vector<MotorConfig> configs_;


    std::mutex cmd_mutex_;

    std::array<MotorCommand, NUM_AXES>   pending_axis_cmds_{};  // ROS intake (axis space)
    std::array<MotorCommand, NUM_AXES>   axis_cmds_{};          // resolved desired (axis space)
    std::array<MotorCommand, NUM_MOTORS> motor_cmds_{};         // transformed (motor space) — only thing framed
    std::array<bool, NUM_MOTORS>         motor_zero_req_{};     // 'd exact 0' side-channel (motor space)
    // -------------------------------------------------------------------------
    // Telemetry (see arm_telemetry.h for struct definition)
    // Updated each poll cycle from CAN reply frames.
    // -------------------------------------------------------------------------
    std::array<MotorTelem, NUM_MOTORS> telem_{};

    // -------------------------------------------------------------------------
    // State tracking for edge-triggered logging (prevents log spam)
    // -------------------------------------------------------------------------
    std::array<int,  NUM_MOTORS> last_fault_{};
    std::array<int,  NUM_MOTORS> last_mode_{};
    std::array<bool, NUM_MOTORS> position_alert_raised_{};



};
