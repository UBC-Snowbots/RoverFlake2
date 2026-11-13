#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <vector>
#include <cmath>
#include <cstdio>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/moteus_config_update.hpp"
#include "std_msgs/msg/string.hpp"


#include <rover_moteus_common/motor_config.h>  
#include "moteus_bus.h"

constexpr int LOG_QUEUE_SIZE_MSGS = 50;

class MoteusNode : public rclcpp::Node {
    public:
        MoteusNode(const std::string& name, const std::string& topic_prefix, std::vector<MotorConfig> configs, std::vector<std::string> motor_names);

        virtual ~MoteusNode() = default; // also run the derived class deconstructors

        // functions that must be implemented in derived classes
        virtual void prepareCommands() = 0;
        virtual void handleTelemetry() = 0;
    private:
        // Startup: push configuration to every motor via "conf set" DiagnosticCommand
        void configureMotors();
        void configureMotor(int motor_id);


        // ROS subscription callback: applies a single register update to one motor
        // via DiagnosticCommand("conf set ...") and updates configs_[] in memory.
        void configUpdateCallback(const rover_msgs::msg::MoteusConfigUpdate::SharedPtr msg);

        // "superloop" style. Blocking can transactions (100hz is slow for CANFD, so this should be okay)
        void run();

        // Apply a (register, value) pair to the in-memory MotorConfig for motor idx.
        // Keeps configs_[] consistent so subsequent feedback messages reflect edits.
        void applyConfigToMemory(int idx, const std::string& reg, float val);

        // Safety monitoring (called inside poll, only logs on state changes)
        void checkFaults();
        void checkAlerts();

        // -------------------------------------------------------------------------
        // ROS interfaces
        // -------------------------------------------------------------------------
        rclcpp::TimerBase::SharedPtr                                      timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                    config_log_pub_;
        rclcpp::Subscription<rover_msgs::msg::MoteusConfigUpdate>::SharedPtr       config_update_sub_;

        // -------------------------------------------------------------------------
        // State tracking for edge-triggered logging (prevents log spam)
        // -------------------------------------------------------------------------
        std::vector<int> last_fault_{}; 
        std::vector<int> last_mode_{};
        std::vector<bool> position_alert_raised_{};

        
    protected: // what only derived classes can see

        // Publish a string to /.../config_log (shown in the HMI command log panel)
        void publishLog(const std::string& msg);

        void zero_position(uint8_t index);
        void set_position(uint8_t index, float position_revs);

        // -------------------------------------------------------------------------
        // CAN transport + per-motor controllers
        // -------------------------------------------------------------------------
        std::unique_ptr<MoteusBus> bus_; 



        // -------------------------------------------------------------------------
        // Configuration (loaded from motor_config.h at construction)
        // -------------------------------------------------------------------------
        std::vector<MotorConfig> configs_;

        std::vector<MotorCommand> motor_cmds_{};         // transformed (motor space) — only thing framed
        std::vector<std::string> motor_names_;

        // -------------------------------------------------------------------------
        // Telemetry (see arm_telemetry.h for struct definition)
        // Updated each poll cycle from CAN reply frames.
        // -------------------------------------------------------------------------
        std::vector<MotorTelem> telem_{};

        // -------------------------------------------------------------------------
        // Calibration state
        // When true, poll() returns immediately so the CAN bus is free for moteus_tool.
        // -------------------------------------------------------------------------
        std::atomic<bool> calibrating_{false};


    
};