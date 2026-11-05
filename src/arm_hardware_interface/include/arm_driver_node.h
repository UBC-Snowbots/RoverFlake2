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
#include <bitset> // for printing out debug bitmasks

#include "rover_msgs/msg/arm_command.hpp"
#include "rover_msgs/msg/moteus_arm_status.hpp"
#include "rover_msgs/msg/bldc_servo_status.hpp"
#include "rover_msgs/msg/bldc_servo_config.hpp"
#include "rover_msgs/msg/moteus_calibration_request.hpp"
#include "rover_msgs/msg/moteus_calibration_status.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// #include <rover_arm_common/motor_addressing.h>   // NUM_MOTORS, ARM_JOINTS, unit converters
#include <rover_arm_common/arm_commands.h>       // CMD_*, MotorCommand
#include <rover_arm_common/arm_telemetry.h>      // MotorTelem
#include "moteus_node.h"


// Half assed debug printout flags - Uncomment to enable these printouts
// #define DEBUG_LIMIT_SWITCH_RAW_REPLY



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

class ArmDriverNode : public MoteusNode {
public:
    ArmDriverNode();

private:
    // ROS subscription callback: fills pending_cmds_[] (mutex-protected)
    void commandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg);

    // Calibration: receives a MoteusCalibrationRequest and spawns a background thread.
    void calibrationCallback(const rover_msgs::msg::MoteusCalibrationRequest::SharedPtr msg);

    // Runs in background thread.  Pauses poll, releases CAN, runs moteus_tool,
    // sets hall sensor type, saves config, re-initialises transport + controllers.
    void runCalibration(int motor_can_id);

    // Publish a calibration status update.
    void publishCalibStatus(int motor_id, int state, const std::string& message);
    
    void home_axis(uint8_t index);

    void prepareCommands() override;
    void handleTelemetry() override;

    // -------------------------------------------------------------------------
    // ROS interfaces
    // -------------------------------------------------------------------------
    rclcpp::Publisher<rover_msgs::msg::MoteusArmStatus>::SharedPtr         feedback_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr             joint_state_pub_;
    rclcpp::Publisher<rover_msgs::msg::ArmCommand>::SharedPtr           arm_feedback_pub;
    rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr           command_sub_;
    //TODO Calibration in future PR
    // rclcpp::Subscription<rover_msgs::msg::MoteusCalibrationRequest>::SharedPtr calib_sub_;
    // rclcpp::Publisher<rover_msgs::msg::MoteusCalibrationStatus>::SharedPtr     calib_pub_;


    std::mutex cmd_mutex_;

    Axis axes[NUM_AXES];
    std::array<MotorCommand, NUM_AXES>   pending_axis_cmds_{};  // ROS intake (axis space)
    std::array<MotorCommand, NUM_AXES>   axis_cmds_{};          // resolved desired (axis space)
    std::array<bool, NUM_MOTORS>         motor_zero_req_{};     // 'd exact 0' side-channel (motor space)
    

    
    std::mutex        calib_mutex_;   // prevents concurrent calibrations




};
