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
//                /joint_states         (sensor_msgs/JointState)  → RViz / RSP
//                /arm/config_log       (std_msgs/String)          → HMI log panel
// =============================================================================

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <array>
#include <bitset> // for printing out debug bitmasks

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/arm_command.hpp"
#include "rover_msgs/msg/moteus_arm_status.hpp"
#include "rover_msgs/msg/bldc_servo_status.hpp"
#include "rover_msgs/msg/bldc_servo_config.hpp"
#include "rover_msgs/msg/moteus_config_update.hpp"
#include "rover_msgs/msg/moteus_calibration_request.hpp"
#include "rover_msgs/msg/moteus_calibration_status.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// #include <rover_arm_common/motor_addressing.h>   // NUM_MOTORS, ARM_JOINTS, unit converters
#include <rover_arm_common/motor_config.h>       // MotorConfig, get_arm_configuration()
#include <rover_arm_common/arm_commands.h>       // CMD_*, MotorCommand
#include <rover_arm_common/arm_telemetry.h>      // MotorTelem
#include "moteus_protocol.h"                       // MoteusProtocol::make*Frame(), parseReply()

enum class AxisState
{
    INIT,
    REQUESTING_HOMING,
    HOMING,
    RUNNING_OK,
    ERROR
};

struct Axis {
        int index; // Starts at 0
        float position = 0;
        bool limit_switch = 0;
        bool homed = false;
        AxisState state = AxisState::INIT;
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

    // 10 Hz poll: build CAN frames → BlockingCycle → parse replies → publish
    void poll();

    // ROS subscription callback: fills pending_cmds_[] (mutex-protected)
    void commandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg);

    // ROS subscription callback: applies a single register update to one motor
    // via DiagnosticCommand("conf set ...") and updates configs_[] in memory.
    void configUpdateCallback(const rover_msgs::msg::MoteusConfigUpdate::SharedPtr msg);

    // Apply a (register, value) pair to the in-memory MotorConfig for motor idx.
    // Keeps configs_[] consistent so subsequent feedback messages reflect edits.
    void applyConfigToMemory(int idx, const std::string& reg, float val);

    // Calibration: receives a MoteusCalibrationRequest and spawns a background thread.
    void calibrationCallback(const rover_msgs::msg::MoteusCalibrationRequest::SharedPtr msg);

    // Runs in background thread.  Pauses poll, releases CAN, runs moteus_tool,
    // sets hall sensor type, saves config, re-initialises transport + controllers.
    void runCalibration(int motor_can_id);

    // Re-create transport_ and controllers_ (called after moteus_tool exits).
    void reInitTransport();

    // Publish a calibration status update.
    void publishCalibStatus(int motor_id, int state, const std::string& message);

    // Safety monitoring (called inside poll, only logs on state changes)
    void checkFaults();
    void checkAlerts();

    // Publish a string to /arm/config_log (shown in the HMI command log panel)
    void publishLog(const std::string& msg);

    void zero_position(MotorIndex index);
    
    void home_axis(AxisIndex index);

    Axis axes[NUM_AXES];

    // -------------------------------------------------------------------------
    // CAN transport + per-motor controllers
    // -------------------------------------------------------------------------
    std::shared_ptr<mot::Transport>              transport_;
    std::vector<std::shared_ptr<mot::Controller>> controllers_;  // index = motor_id - 1

    // -------------------------------------------------------------------------
    // ROS interfaces
    // -------------------------------------------------------------------------
    rclcpp::TimerBase::SharedPtr                                      timer_;
    rclcpp::Publisher<rover_msgs::msg::MoteusArmStatus>::SharedPtr         feedback_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr             joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                    config_log_pub_;
    rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr           command_sub_;
    rclcpp::Subscription<rover_msgs::msg::MoteusConfigUpdate>::SharedPtr       config_update_sub_;
    rclcpp::Subscription<rover_msgs::msg::MoteusCalibrationRequest>::SharedPtr calib_sub_;
    rclcpp::Publisher<rover_msgs::msg::MoteusCalibrationStatus>::SharedPtr     calib_pub_;

    // -------------------------------------------------------------------------
    // Configuration (loaded from motor_config.h at construction)
    // -------------------------------------------------------------------------
    std::vector<MotorConfig> configs_;

    // -------------------------------------------------------------------------
    // Command pipeline (see arm_commands.h for struct definition)
    //
    //   pending_cmds_ — written by commandCallback() under cmd_mutex_
    //                   cleared after each poll merges them into active_cmds_
    //
    //   active_cmds_  — re-sent every poll cycle to keep the watchdog alive
    //                   stays set until a stop or override arrives
    // -------------------------------------------------------------------------
    std::mutex cmd_mutex_;
    std::array<MotorCommand, NUM_MOTORS> pending_cmds_{};
    std::array<MotorCommand, NUM_MOTORS> active_cmds_{};

    // -------------------------------------------------------------------------
    // Telemetry (see arm_telemetry.h for struct definition)
    // Updated each poll cycle from CAN reply frames.
    // -------------------------------------------------------------------------
    std::array<MotorTelem, NUM_MOTORS> telem_{};

    // -------------------------------------------------------------------------
    // Calibration state
    // When true, poll() returns immediately so the CAN bus is free for moteus_tool.
    // -------------------------------------------------------------------------
    std::atomic<bool> calibrating_{false};
    std::mutex        calib_mutex_;   // prevents concurrent calibrations

    // -------------------------------------------------------------------------
    // State tracking for edge-triggered logging (prevents log spam)
    // -------------------------------------------------------------------------
    std::array<int,  NUM_MOTORS> last_fault_{};
    std::array<int,  NUM_MOTORS> last_mode_{};
    std::array<bool, NUM_MOTORS> position_alert_raised_{};



};
