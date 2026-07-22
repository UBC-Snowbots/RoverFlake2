#include "moteus_driver_node.h"
#include "axis_5_6_differential.h"
#include <cmath>
#include <cstdlib>
#include <thread>
#include <cstdio>      // popen / pclose
#include <sys/wait.h>  // WIFEXITED / WEXITSTATUS
#include <stdexcept>

#define LOG_QUEUE_SIZE_MSGS 50
#define SKIP_CALIBRATION
using namespace std::chrono_literals;

// =============================================================================
// MoteusDriverNode implementation
//
// For protocol details, units, and data field contents see moteus_protocol.h.
// For joint mapping and direction signs see motor_addressing.h.
// For per-motor PID/limits see motor_config.h.
// =============================================================================

MoteusDriverNode::MoteusDriverNode() : Node("moteus_driver") {
    // Use a non-singleton transport so we own the only shared_ptr and can truly
    // release the CAN fd when calibration needs exclusive access to the bus.
    // transport_ = mot::TransportRegistry::singleton().make({}).first;
    configs_   = get_arm_configuration();

    for(int i = 0; i < NUM_AXES; i++)
    {
        auto& axis = axes[i];
        axis.homed = false;
        axis.index = i;
        axis.state = AxisState::INIT;

        axis.max_position_rev   = AxisConfig::max_position_rev[i];
        axis.min_position_rev   = AxisConfig::min_position_rev[i];
        axis.homing_speed_revps = AxisConfig::homing_speed_revps[i];
        axis.homing_direction   = AxisConfig::homing_direction[i];


    }
    //ROWTAG deslopify - repeated code
    reInitTransport();
    // for (int id = 1; id <= NUM_MOTORS; id++) {
    //     mot::Controller::Options opts;
    //     opts.id        = id;
    //     opts.transport = transport_;  // explicit — prevents fallback to singleton
    //     opts.query_format.q_current = mot::kFloat;
    //     opts.query_format.power     = mot::kFloat;
    //     controllers_.push_back(std::make_shared<mot::Controller>(opts));
    // }

    auto qos     = rclcpp::QoS(5).reliable().durability_volatile();
    auto log_qos = rclcpp::QoS(LOG_QUEUE_SIZE_MSGS).reliable().durability_volatile();

    feedback_pub_  = this->create_publisher<rover_msgs::msg::MoteusArmStatus>("/arm/moteus_feedback", qos);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);
    config_log_pub_  = this->create_publisher<std_msgs::msg::String>("/arm/config_log", log_qos);
    arm_feedback_pub = this->create_publisher<rover_msgs::msg::ArmCommand>("/arm/feedback", qos);
    

    command_sub_ = this->create_subscription<rover_msgs::msg::ArmCommand>(
        "/arm/command", qos,
        std::bind(&MoteusDriverNode::commandCallback, this, std::placeholders::_1));

    config_update_sub_ = this->create_subscription<rover_msgs::msg::MoteusConfigUpdate>(
        "/arm/config_update", qos,
        std::bind(&MoteusDriverNode::configUpdateCallback, this, std::placeholders::_1));

    calib_pub_ = this->create_publisher<rover_msgs::msg::MoteusCalibrationStatus>(
        "/arm/calibration_status", qos);

    // calib_sub_ = this->create_subscription<rover_msgs::msg::MoteusCalibrationRequest>(
    //     "/arm/calibration_request", qos,
    //     std::bind(&MoteusDriverNode::calibrationCallback, this, std::placeholders::_1));

        #ifndef SKIP_CALIBRATION
        // configureMotors();
        #else 
        RCLCPP_WARN(this->get_logger(), "SKIPPING MOTOR CALIBRATIONS");
        #endif

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&MoteusDriverNode::poll, this));

    RCLCPP_INFO(this->get_logger(),
        "Moteus driver started: polling %d motors at 100 Hz", NUM_MOTORS);
}


// ---------------------------------------------------------------------------
// Startup configuration — pushes MotorConfig values to firmware
// ---------------------------------------------------------------------------

void MoteusDriverNode::publishLog(const std::string& msg) {
    std_msgs::msg::String m;
    m.data = msg;
    config_log_pub_->publish(m);
}

void MoteusDriverNode::configureMotors() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        configureMotor(i + 1, *controllers_[i]);

        auto& c = configs_[i];
        publishLog(std::string("# Motor ") + std::to_string(i + 1)
            + " (" + ARM_JOINTS[i].hardware_name + ") configured:"
            + " kp=" + c.format(c.kp)
            + " kd=" + c.format(c.kd)
            + " max_current=" + c.format(c.max_current_A) + "A"
            + " pos=[" + c.format(c.position_min) + ", " + c.format(c.position_max) + "]rev"
            + " max_vel=" + c.format(c.max_velocity) + "rev/s"
            + " gear=" + c.format(c.gear_reduction));
    }
}

void MoteusDriverNode::configureMotor(int motor_id, mot::Controller& ctrl) {
    auto maybe_state = ctrl.SetQuery();
    if (!maybe_state) {
        RCLCPP_WARN(this->get_logger(),
            "Motor %d (%s) NOT CONNECTED. Skipping config.",
            motor_id, ARM_JOINTS[motor_id - 1].hardware_name);
        publishLog("# Motor " + std::to_string(motor_id)
            + " (" + ARM_JOINTS[motor_id - 1].hardware_name + ") NOT CONNECTED — skipped");
        return;
    }

    RCLCPP_INFO(this->get_logger(),
        "Motor %d (%s) detected. Pushing config...",
        motor_id, ARM_JOINTS[motor_id - 1].hardware_name);

    for (const auto& [reg, val] : configs_[motor_id - 1].get_configs()) {
        auto reply = ctrl.DiagnosticCommand("conf set " + reg + " " + val);
        RCLCPP_INFO(this->get_logger(), "[%d] %s = %s (reply: %s)",
            motor_id, reg.c_str(), val.c_str(), reply.c_str());
    }
}


// ---------------------------------------------------------------------------
// Command callback — ROS topic → pending_cmds_[]
// (See arm_commands.h for command code definitions and MotorCommand struct.)
// ---------------------------------------------------------------------------

void MoteusDriverNode::commandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    char cmd_type = msg->cmd_type;

    if (cmd_type == CMD_STOP) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            pending_cmds_[i].active  = true;
            pending_cmds_[i].is_stop = true;
        }
        RCLCPP_INFO(this->get_logger(), "Command: STOP ALL");
        return;
    }

    if (cmd_type == CMD_ABS_POS) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            double pos = (i < (int)msg->positions.size())  ? msg->positions[i]  : NAN;
            double vel_degrees = (i < (int)msg->velocities.size()) ? msg->velocities[i] : NAN;
            if (std::isnan(pos) && std::isnan(vel_degrees)) continue;

            //TODO fault handling
            // if (telem_[i].fault != 0) {
            //     RCLCPP_WARN(this->get_logger(),
            //         "BLOCKED position cmd to motor %d (%s) — fault %d active. "
            //         "Send STOP first to clear.",
            //         i + 1, ARM_JOINTS[i].hardware_name, telem_[i].fault);
            //     publishLog("# BLOCKED cmd to motor " + std::to_string(i + 1)
            //         + " (" + ARM_JOINTS[i].hardware_name
            //         + ") — fault " + std::to_string(telem_[i].fault));
            //     continue;
            // }

            pending_cmds_[i].active    = true;
            pending_cmds_[i].is_stop   = false;
            pending_cmds_[i].position  = pos;
            pending_cmds_[i].velocity  = degreesToRevolution(vel_degrees);
            pending_cmds_[i].max_torque = NAN;
        }
        return;
    }

    if (cmd_type == CMD_ABS_VEL) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            double vel_degrees = (i < (int)msg->velocities.size()) ? msg->velocities[i] : NAN;
            if (std::isnan(vel_degrees)) continue;

            // if (vel != 0.0 && telem_[i].fault != 0) {
            //     RCLCPP_WARN(this->get_logger(),
            //         "BLOCKED velocity cmd to motor %d (%s) — fault %d active. "
            //         "Send STOP first to clear.",
            //         i + 1, ARM_JOINTS[i].hardware_name, telem_[i].fault);
            //     publishLog("# BLOCKED cmd to motor " + std::to_string(i + 1)
            //         + " (" + ARM_JOINTS[i].hardware_name
            //         + ") — fault " + std::to_string(telem_[i].fault));
            //     continue;
            // }

            pending_cmds_[i].active  = true;
            // if (vel == 0.0) {
                // pending_cmds_[i].is_stop = true;
            // } else {
                pending_cmds_[i].is_stop  = false;
                pending_cmds_[i].position = NAN;
                pending_cmds_[i].velocity = degreesToRevolution(vel_degrees);
                pending_cmds_[i].max_velocity = degreesToRevolution(vel_degrees);
                pending_cmds_[i].max_torque = NAN;
            // }
        }
        return;
    }

    if (cmd_type == CMD_ZERO) {
        // positions[] used as a flag: non-NaN entry → zero that motor
        for (int i = 0; i < NUM_MOTORS; i++) {
            double flag = (i < (int)msg->positions.size()) ? msg->positions[i] : NAN;
            if (!std::isnan(flag)) {
                pending_cmds_[i].active  = true;
                pending_cmds_[i].is_zero = true;
            }
        }
        return;
    }

    if (cmd_type == CMD_HOME)
    {
        // Sanitize cmd_value
        if(msg->cmd_value >= 0 && msg->cmd_value < NUM_AXES)
        {
            // User wants to home a single axis
            home_axis(static_cast<AxisIndex>(msg->cmd_value));
            return;
        } 
        else if(msg->cmd_value == HOME_VALUE_ALL_AXES_EXCEPT_EE)
        {
            // User wants to home all axes, except for EE
            home_axis(AxisIndex::AXIS_5);
        }
        else if(msg->cmd_value == HOME_VALUE_ALL_AXES_AND_EE)
        {
            // User wants to home all axes including EE
        }
        else 
        {
            // Invalid command. Return.
            RCLCPP_WARN(this->get_logger(), "Unknown home command value: %i", msg->cmd_value);
            return;
        }
    }

    RCLCPP_WARN(this->get_logger(), "Unknown cmd_type: '%c' (%d)", cmd_type, (int)cmd_type);
}

// Zero position of a motor.
void MoteusDriverNode::zero_position(MotorIndex index)
{
    int i = static_cast<int>(index);

    controllers_[i]->DiagnosticCommand("d exact 0");
    RCLCPP_INFO(this->get_logger(),
    "Motor %d (%s) position set to zero (d exact 0)", i + 1, ARM_JOINTS[i].hardware_name);
    publishLog("# Motor " + std::to_string(i + 1)
        + " (" + ARM_JOINTS[i].hardware_name + ") zeroed");
}

void MoteusDriverNode::set_position(MotorIndex index, float position_revs)
{
  int i = static_cast<int>(index);

  char cmd[32];
  std::snprintf(cmd, sizeof(cmd), "d exact %f", position_revs);
  controllers_[i]->DiagnosticCommand(cmd);
}

void MoteusDriverNode::home_axis(AxisIndex index)
{
    // Just set axis state to request homing
    this->axes[static_cast<int>(index)].state = AxisState::REQUESTING_HOMING;
}

// ---------------------------------------------------------------------------
// Poll loop — runs at 100 Hz
//
// Step 1: merge pending_cmds_ into active_cmds_  (mutex-protected, O(N))
// Step 2: build one CAN frame per motor           (see moteus_protocol.h)
// Step 3: BlockingCycle — send all frames, wait for all replies
// Step 4: decode replies into telem_[]            (see arm_telemetry.h)
// Step 5: safety checks                           (checkFaults, checkAlerts)
// Step 6: publish /arm/moteus_feedback and /joint_states
// ---------------------------------------------------------------------------

void MoteusDriverNode::poll() {
    // Yield CAN bus to the calibration thread while it runs moteus_tool.
    if (calibrating_.load()) return;

    // Step 1 — merge
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (pending_cmds_[i].active && axes[i].state != AxisState::HOMING)
                active_cmds_[i] = pending_cmds_[i];
        }
        pending_cmds_ = {};
    }

    // Step 2a — "d exact 0" zero commands (diagnostic channel, separate from BlockingCycle)
    // This resets the position counter to 0 at the current physical position — no movement.
    for (int i = 0; i < NUM_MOTORS; i++) {
        auto& cmd = active_cmds_[i];
        if (cmd.active && cmd.is_zero) {
            zero_position(static_cast<MotorIndex>(i));
            cmd.active = false;  // one-shot; next cycle falls through to MakeQuery
        }
    }
    
    // Check Homing
    for(auto& axis : axes)
    {
        if(axis.state == AxisState::REQUESTING_HOMING)
        {
            if(axis.limit_switch)
            {
                // Limit swittch is pressed or broken
                RCLCPP_INFO(this->get_logger(), "Axis %i Limit Switch Is pressed or broken. Homing Request Denied", axis.index + 1);
                axis.state = AxisState::ERROR;

            } else 
            {
                RCLCPP_INFO(this->get_logger(), "Axis %i Limit Switch appears healthy. Homing Request Accepted", axis.index + 1);

                if(axis.index == 4 || axis.index == 5)
                {
                    // set_position(static_cast<MotorIndex>(4), AxisConfig::max_position_rev[4] - 0.01); //sloppy
                    // set_position(static_cast<MotorIndex>(5), AxisConfig::max_position_rev[5] - 0.01); //sloppy

                } else {
                    set_position(static_cast<MotorIndex>(axis.index), AxisConfig::max_position_rev[axis.index] - 0.01); //sloppy
                }

                // Accept
                axis.state = AxisState::HOMING;
            }
        }
        
        if(axis.state == AxisState::GOING_TO_PRESET_POSITION)
        {
            if(axis.index == 5)
            {
                // Axis 5 drives to idle; axis 6 holds current (it isn't homed,
                // so an absolute idle_position[5] target is meaningless).
                auto& c4 = active_cmds_[4];
                c4.active = true;  c4.is_stop = false;  c4.is_zero = false;
                c4.position = AxisConfig::idle_position[4];   // real axis-space target
                c4.velocity = NAN;
                c4.max_velocity = 0.1f;  c4.max_acceleration = 0.1f;

                auto& c5 = active_cmds_[5];
                c5.active = true;  c5.is_stop = false;  c5.is_zero = false;
                c5.position = axes[5].position;               // HOLD, don't chase idle
                c5.velocity = NAN;
                c5.max_velocity = 0.1f;  c5.max_acceleration = 0.1f;

                // arrival keyed on axis 5 only — axis 6 isn't going anywhere
                if(std::fabs(axes[4].position - AxisConfig::idle_position[4]) < 0.01f)
                {
                    axes[4].state = AxisState::RUNNING_OK;
                    // axes[5].state = AxisState::RUNNING_OK;
                }
            }
            else
            {
                auto& cmd = active_cmds_[axis.index];
                cmd.active = true;  cmd.is_stop = false;  cmd.is_zero = false;
                cmd.position = AxisConfig::idle_position[axis.index];
                cmd.velocity = NAN;
                cmd.max_velocity = 0.1f;  cmd.max_acceleration = 0.1f;
                if(std::fabs(axes[axis.index].position - AxisConfig::idle_position[axis.index]) < 0.01f)
                    axes[axis.index].state = AxisState::RUNNING_OK;
            }
        }

        if(axis.state == AxisState::HOMING)
        {
            // Check limit switch
            if(axis.limit_switch )
            {
                // Limit switch is pressed, homing complete
                RCLCPP_INFO(this->get_logger(), "Axis %i Limit Switch Triggered. Homing Complete!", axis.index + 1);

                if(axis.index == 4 || axis.index == 5)
                {
                    zero_position(MotorIndex::MOTOR_5);
                     zero_position(MotorIndex::MOTOR_6);

                } else {
                     zero_position(static_cast<MotorIndex>(axis.index));

                }
                axis.state = AxisState::GOING_TO_PRESET_POSITION;


                int i = axis.index;
                if(i == static_cast<int>(AxisIndex::AXIS_5))
                {
                    axis.state = AxisState::GOING_TO_PRESET_POSITION;
                    // axes[5].state = AxisState::GOING_TO_PRESET_POSITION;
                    // int m5_i = static_cast<int>(MotorIndex::MOTOR_5);
                    // int m6_i = static_cast<int>(MotorIndex::MOTOR_6);
                    // // Special case for differential axes
                    // float target_5 = (AxisConfig::idle_position[axis.index]  * AxisConfig::homing_direction[m5_i] * -1);
                    // auto& cmd_5      = active_cmds_[m5_i];
                    // cmd_5.active     = true;
                    // cmd_5.is_stop    = false;
                    // cmd_5.is_zero    = false;
                    // cmd_5.position   = target_5;
                    // cmd_5.max_acceleration = 0.1;
                    // cmd_5.max_velocity = 0.1;
                    // cmd_5.max_torque = NAN;

                    // // MUST set the axis-6 half so the transform's 2nd input is real, not stale.
                    // active_cmds_[5].active   = true;
                    // active_cmds_[5].is_stop  = false;
                    // active_cmds_[5].is_zero  = false;
                    // active_cmds_[5].position = axes[5].position;   // hold
                    // active_cmds_[5].velocity = NAN;
                    // active_cmds_[5].max_velocity     = AxisConfig::homing_speed_revps[5];
                    // active_cmds_[5].max_acceleration = 0.1;

                    // float target_6 =  (AxisConfig::idle_position[m6_i]  * AxisConfig::homing_direction[m5_i] * -1);
                    // auto& cmd_6      = active_cmds_[m6_i];
                    // cmd_6.active     = true;
                    // cmd_6.is_stop    = false;
                    // cmd_6.is_zero    = false;
                    // cmd_6.position   = target_6; //telem_[m6_i].position; 
                    // // cmd_6.velocity   = 0.0;
                    // cmd_6.max_acceleration = 0.1;
                    // cmd_6.max_velocity = 0.1;
                    // cmd_6.max_torque = NAN;
                }
                else if (i == static_cast<int>(AxisIndex::AXIS_6))
                {

                }
                else
                {
                float target = AxisConfig::idle_position[axis.index] * -1 * AxisConfig::homing_direction[axis.index];
  
                auto& cmd      = active_cmds_[axis.index];
                cmd.active     = true;
                cmd.is_stop    = false;
                cmd.is_zero    = false;
                cmd.position   = target;
                cmd.velocity   = NAN;
                // cmd.velocity   = 0.01; // Prob doesn't do what you think, use max_velocity instead below
                cmd.max_acceleration = 0.1;
                cmd.max_velocity = 0.1;
                cmd.max_torque = NAN;



                }

            } 
            else 
            {
                // Move the motor a little tiny bit
                constexpr float kHomingStepRev = 0.05f;  // ~0.7 deg motor/cycle

                int i = axis.index;
                if(i == static_cast<int>(AxisIndex::AXIS_5))
                {
                        // Axis space in, Step 2b does the ONE transform to motor space.
                        // Drive axis 5 toward its switch; hold axis 6.
                        active_cmds_[4].active   = true;
                        active_cmds_[4].is_stop  = false;
                        active_cmds_[4].is_zero  = false;
                        active_cmds_[4].position = axes[4].position + (kHomingStepRev * axis.homing_direction);
                        active_cmds_[4].velocity = NAN;
                        active_cmds_[4].max_velocity     = AxisConfig::homing_speed_revps[4];
                        active_cmds_[4].max_acceleration = 0.1;

                        // MUST set the axis-6 half so the transform's 2nd input is real, not stale.
                        active_cmds_[5].active   = true;
                        active_cmds_[5].is_stop  = false;
                        active_cmds_[5].is_zero  = false;
                        active_cmds_[5].position = axes[5].position;   // hold
                        active_cmds_[5].velocity = NAN;
                        active_cmds_[5].max_velocity     = AxisConfig::homing_speed_revps[5];
                        active_cmds_[5].max_acceleration = 0.1;
                    // int m5_i = static_cast<int>(MotorIndex::MOTOR_5);
                    // int m6_i = static_cast<int>(MotorIndex::MOTOR_6);
                    // // Special case for differential axes
                    // float target_5 = telem_[m5_i].position + (kHomingStepRev * AxisConfig::homing_direction[m5_i]);
                    // auto& cmd_5      = active_cmds_[m5_i];
                    // cmd_5.active     = true;
                    // cmd_5.is_stop    = false;
                    // cmd_5.is_zero    = false;
                    // cmd_5.position   = target_5;
                    // cmd_5.max_acceleration = 0.1;
                    // cmd_5.max_velocity = AxisConfig::homing_speed_revps[m5_i];
                    // cmd_5.max_torque = NAN;

                // Tell axis 6 to hold position
                    // float target_6 = telem_[m6_i].position; //+ (kHomingStepRev * AxisConfig::homing_direction[m5_i]); // Motor 5 and 6 directions are flipped, but we still need to refrence based on Axis 5.
                    // auto& cmd_6      = active_cmds_[m6_i];
                    // cmd_6.active     = true;
                    // cmd_6.is_stop    = false;
                    // cmd_6.is_zero    = false;
                    // cmd_6.position   = target_6; //telem_[m6_i].position; 
                    // // cmd_6.velocity   = 0.0;
                    // cmd_6.max_acceleration = 0.1;
                    // cmd_6.max_velocity = AxisConfig::homing_speed_revps[m6_i];
                    // cmd_6.max_torque = NAN;
                }
                else if (i == static_cast<int>(AxisIndex::AXIS_6))
                {

                }
                else
                {
                    float target = telem_[axis.index].position + (kHomingStepRev * axis.homing_direction);
                    auto& cmd      = active_cmds_[i];
                    cmd.active     = true;
                    cmd.is_stop    = false;
                    cmd.is_zero    = false;
                    cmd.position   = target;
                    cmd.max_acceleration = 0.1;
                    cmd.max_velocity = AxisConfig::homing_speed_revps[axis.index];
                    cmd.max_torque = NAN;
                }

            }

        }
    }

    // Step 2b — apply differential wrist transform for axes 5 & 6 (indices 4 & 5)
    // The wrist is mechanically coupled: joint-space commands must be converted
    // to motor-space commands before sending.  See axis_5_6_differential.h.
    if ((active_cmds_[4].active || active_cmds_[5].active) &&
        !active_cmds_[4].is_stop && !active_cmds_[5].is_stop)
    {
        // Force both motors to be live
        active_cmds_[4].active = true;
        active_cmds_[5].active = true;

        if(axes[4].state == AxisState::HOMING || axes[5].state == AxisState::HOMING ||
            axes[4].state == AxisState::GOING_TO_PRESET_POSITION || axes[5].state == AxisState::GOING_TO_PRESET_POSITION)
        {

            float m5, m6;
            differential_drive(
                static_cast<float>(active_cmds_[4].position),
                static_cast<float>(active_cmds_[5].position),
                m5, m6);
            active_cmds_[4].velocity = NAN;
            active_cmds_[5].velocity = NAN;

            active_cmds_[4].position = m5;
            active_cmds_[5].position = m6;

            active_cmds_[4].max_acceleration = 0.1;
            active_cmds_[4].max_velocity = AxisConfig::homing_speed_revps[4];
            
            active_cmds_[5].max_velocity = AxisConfig::homing_speed_revps[5];
            active_cmds_[5].max_acceleration = 0.1;
        } else {
            // Swap back to velocity control
            float m5, m6;
            differential_drive(
                static_cast<float>(active_cmds_[4].velocity),
                static_cast<float>(active_cmds_[5].velocity),
                m5, m6);
            active_cmds_[4].velocity = m5;
            active_cmds_[5].velocity = m6;

            active_cmds_[4].max_acceleration = AxisConfig::max_running_accel[4];
            active_cmds_[4].max_velocity = AxisConfig::max_running_speed[4];
            
            active_cmds_[5].max_velocity = AxisConfig::homing_speed_revps[5];
            active_cmds_[5].max_acceleration = AxisConfig::max_running_accel[5];
        }
    }

    // Step 2c — build frames
    // (See moteus_protocol.h for what goes in the data field of each type.)
    std::vector<mot::CanFdFrame> frames;
    for (int i = 0; i < NUM_MOTORS; i++) {
        auto& cmd = active_cmds_[i];
        if (cmd.active && cmd.is_stop) {
            frames.push_back(MoteusProtocol::makeStopFrame(*controllers_[i]));
            cmd.active = false;  // stop is one-shot; next cycle falls through to query
        } else if (cmd.active) {
            // frames.push_back(MoteusProtocol::makePositionFrame(
                // *controllers_[i], cmd.position, NAN));
            frames.push_back(MoteusProtocol::makePositionFrame(
                *controllers_[i], cmd.position, cmd.velocity, cmd.max_velocity, cmd.max_acceleration));
            cmd.active = false; 

        } else {
            frames.push_back(MoteusProtocol::makeQueryFrame(*controllers_[i]));
        }
    }

    // Step 3 — send + receive
    std::vector<mot::CanFdFrame> replies;
    transport_->BlockingCycle(frames.data(), frames.size(), &replies);

    // Step 4 — decode replies
    // Reset connected flags; only set true for motors that replied this cycle.
    for (auto& t : telem_) t.connected = false;

    for (const auto& frame : replies) {
        int id = frame.source;
        if (id < 1 || id > NUM_MOTORS) continue;

        auto r = MoteusProtocol::parseReply(frame);
        auto& t = telem_[id - 1];
        t.position    = r.position;
        t.velocity    = r.velocity;
        t.torque      = r.torque;
        t.voltage     = r.voltage;
        t.temperature = r.temperature;
        t.q_current   = std::isnan(r.q_current) ? 0.0f : static_cast<float>(r.q_current);
        t.power       = std::isnan(r.power)     ? 0.0f : static_cast<float>(r.power);
        t.mode        = static_cast<int>(r.mode);
        t.fault       = static_cast<int>(r.fault);
        t.connected   = true;
        #ifdef DEBUG_LIMIT_SWITCH_RAW_REPLY
        RCLCPP_WARN(this->get_logger(), "Aux2 Pins of axis %i be looking like: %s",
            id,
            std::bitset<8>(r.aux2_gpio).to_string().c_str());
        #endif
        t.limit_switch = (r.aux2_gpio > 0); //TODO this is pretty fragile... if pins are not configured then is fucky wucky
        axes[id - 1].limit_switch = t.limit_switch; // Update for IPC
        if((id -1) != static_cast<int>(MotorIndex::MOTOR_5) && (id - 1) != static_cast<int>(MotorIndex::MOTOR_6))
        {
            axes[id - 1].position = t.position;

        } else {
            // Handle motor 5 / 6, but wait for both motors data to update
            if((id - 1) == static_cast<int>(MotorIndex::MOTOR_6))
            {
                float a5, a6;
                differential_drive_inverse(telem_[4].position, telem_[5].position, a5, a6);
                axes[4].position = a5;
                axes[5].position = a6;
            }
        }
    }

    // Step 5 — safety
    // checkFaults();
    checkAlerts();

    // publish to arm feedback (not moteus feedback, bandaid)
    rover_msgs::msg::ArmCommand arm_feedback_msg;
    arm_feedback_msg.positions.resize(NUM_AXES);
    arm_feedback_msg.velocities.resize(NUM_AXES);
    for(auto axis : axes)
    {
        arm_feedback_msg.positions[axis.index] = axis.position;
    }
    arm_feedback_pub->publish(arm_feedback_msg);

    // Step 6a — /arm/moteus_feedback
    rover_msgs::msg::MoteusArmStatus status_msg;
    status_msg.status.resize(NUM_MOTORS);
    status_msg.config.resize(NUM_MOTORS);
    status_msg.limit_switches.resize(NUM_MOTORS);

    for (int i = 0; i < NUM_MOTORS; i++) {
        auto& c   = configs_[i];
        auto& cfg = status_msg.config[i];
        cfg.max_acceleration  = c.max_acceleration;
        cfg.max_velocity      = c.max_velocity;
        cfg.max_position      = c.position_max;
        cfg.min_position      = c.position_min;
        cfg.kp                = c.kp;
        cfg.ki                = c.ki;
        cfg.kd                = c.kd;
        cfg.max_current_amps  = c.max_current_A;
        cfg.max_voltage_volts = c.max_voltage;
        cfg.max_power_watts   = c.max_power_W;
        cfg.cmd_timeout_s     = c.def_timeout;
        cfg.gear_reduction    = c.gear_reduction;


        auto& t = telem_[i];
        auto& s = status_msg.status[i];
        s.curr_position        = t.position;
        s.curr_velocity        = t.velocity;
        s.curr_torque          = t.torque;
        s.curr_voltage_volts   = t.voltage;
        s.curr_current_amps    = t.q_current;
        s.curr_power_watts     = t.power;
        s.driver_temp_degreesc = t.temperature;
        s.moteus_mode          = static_cast<int16_t>(t.mode);
        s.moteus_fault         = static_cast<int16_t>(t.fault);
        // Desired setpoints from the last command sent to this motor
        s.des_position = static_cast<float>(active_cmds_[i].active ? active_cmds_[i].position : NAN);
        s.des_velocity = static_cast<float>(active_cmds_[i].active ? active_cmds_[i].velocity : NAN);
        status_msg.limit_switches[i] = t.limit_switch;
    }
    feedback_pub_->publish(status_msg);

    // Step 6b — /joint_states (consumed by robot_state_publisher → RViz2)
    // Unit conversion via motorRevToJointRad() — see motor_addressing.h.
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name.resize(NUM_MOTORS + NUM_GRIPPER_JOINTS);
    js.position.resize(NUM_MOTORS + NUM_GRIPPER_JOINTS);
    js.velocity.resize(NUM_MOTORS + NUM_GRIPPER_JOINTS);

    for (int i = 0; i < NUM_MOTORS; i++) {
        js.name[i]     = ARM_JOINTS[i].urdf_joint_name;
        js.position[i] = motorRevToJointRad(i, telem_[i].position);
        js.velocity[i] = motorRevPerSecToJointRadPerSec(i, telem_[i].velocity);
    }
    for (int g = 0; g < NUM_GRIPPER_JOINTS; g++) {
        js.name[NUM_MOTORS + g]     = GRIPPER_JOINT_NAMES[g];
        js.position[NUM_MOTORS + g] = 0.0;
        js.velocity[NUM_MOTORS + g] = 0.0;
    }
    joint_state_pub_->publish(js);
}


// ---------------------------------------------------------------------------
// Fault detection — edge-triggered (logs only on state change, not every cycle)
// ---------------------------------------------------------------------------

void MoteusDriverNode::checkFaults() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        auto& t = telem_[i];
        if (!t.connected) continue;

        if (t.fault != last_fault_[i]) {
            if (t.fault != 0) {
                RCLCPP_ERROR(this->get_logger(),
                    "Motor %d (%s) FAULT! Code: %d — commands blocked until STOP sent",
                    i + 1, ARM_JOINTS[i].hardware_name, t.fault);
                publishLog("# FAULT motor " + std::to_string(i + 1)
                    + " (" + ARM_JOINTS[i].hardware_name
                    + ") code=" + std::to_string(t.fault));
            } else if (last_fault_[i] != 0) {
                RCLCPP_INFO(this->get_logger(),
                    "Motor %d (%s) fault cleared", i + 1, ARM_JOINTS[i].hardware_name);
                publishLog("# Motor " + std::to_string(i + 1)
                    + " (" + ARM_JOINTS[i].hardware_name + ") fault cleared");
            }
            last_fault_[i] = t.fault;
        }

        if (t.mode != last_mode_[i]) {
            if (t.mode == 1)
                RCLCPP_ERROR(this->get_logger(),
                    "Motor %d (%s) entered FAULT mode", i + 1, ARM_JOINTS[i].hardware_name);
            else if (t.mode == 0 && last_mode_[i] != 0)
                RCLCPP_WARN(this->get_logger(),
                    "Motor %d (%s) is STOPPED (driver disabled)", i + 1, ARM_JOINTS[i].hardware_name);
            last_mode_[i] = t.mode;
        }
    }
}


// ---------------------------------------------------------------------------
// Position limit alerts — warns when approaching configured bounds
// ---------------------------------------------------------------------------

void MoteusDriverNode::checkAlerts() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        auto& t = telem_[i];
        if (!t.connected) continue;

        auto&  c       = configs_[i];
        float  padding = c.position_warn_rev_padding;
        bool near_max  = t.position >= (c.position_max - padding);
        bool near_min  = t.position <= (c.position_min + padding);

        if (near_max || near_min) {
            if (!position_alert_raised_[i]) {
                position_alert_raised_[i] = true;
                const char* which = near_max ? "MAX" : "MIN";
                float limit = near_max ? c.position_max : c.position_min;
                RCLCPP_WARN(this->get_logger(),
                    "Motor %d (%s) near %s position limit! pos=%.3f limit=%.3f (padding=%.3f)",
                    i + 1, ARM_JOINTS[i].hardware_name, which,
                    t.position, limit, padding);
                publishLog("# WARNING motor " + std::to_string(i + 1)
                    + " (" + ARM_JOINTS[i].hardware_name + ") near " + which
                    + " limit: pos=" + std::to_string(t.position)
                    + " limit=" + std::to_string(limit));
            }
        } else {
            position_alert_raised_[i] = false;
        }
    }
}


// ---------------------------------------------------------------------------
// Real-time config update — HMI publishes to /arm/config_update
//
// Applies a single "conf set <register> <value>" to the requested motor and
// updates the in-memory MotorConfig so subsequent feedback messages reflect
// the change.  The moteus firmware applies the value immediately (RAM only —
// not persisted to flash unless you separately run "conf write").
// ---------------------------------------------------------------------------

void MoteusDriverNode::configUpdateCallback(
        const rover_msgs::msg::MoteusConfigUpdate::SharedPtr msg)
{
    int idx = msg->motor_id;
    if (idx < 0 || idx >= NUM_MOTORS) {
        RCLCPP_WARN(this->get_logger(),
            "configUpdate: motor_id %d out of range (0..%d)", idx, NUM_MOTORS - 1);
        return;
    }
    if (msg->register_name.empty() || std::isnan(msg->value)) return;

    const std::string val_str = std::to_string(msg->value);
    const std::string cmd = "conf set " + msg->register_name + " " + val_str;

    RCLCPP_INFO(this->get_logger(),
        "configUpdate motor %d (%s): %s = %s",
        idx + 1, ARM_JOINTS[idx].hardware_name,
        msg->register_name.c_str(), val_str.c_str());
    publishLog("# conf motor " + std::to_string(idx + 1)
        + " (" + ARM_JOINTS[idx].hardware_name + "): "
        + msg->register_name + " = " + val_str);

    controllers_[idx]->DiagnosticCommand(cmd);

    // For max_velocity, the moteus firmware has two registers that must both
    // be updated to cap velocity consistently during motion profile planning.
    if (msg->register_name == "servo.max_velocity") {
        controllers_[idx]->DiagnosticCommand(
            "conf set servo.default_velocity_limit " + val_str);
    } else if (msg->register_name == "servo.default_velocity_limit") {
        controllers_[idx]->DiagnosticCommand(
            "conf set servo.max_velocity " + val_str);
    }

    applyConfigToMemory(idx, msg->register_name, msg->value);
}

void MoteusDriverNode::applyConfigToMemory(int idx, const std::string& reg, float val) {
    auto& c = configs_[idx];
    if      (reg == "servo.pid_position.kp")        c.kp              = val;
    else if (reg == "servo.pid_position.ki")        c.ki              = val;
    else if (reg == "servo.pid_position.kd")        c.kd              = val;
    else if (reg == "servo.max_current_A")          c.max_current_A   = val;
    else if (reg == "servo.max_velocity"
          || reg == "servo.default_velocity_limit") c.max_velocity    = val;
    else if (reg == "servo.default_accel_limit")    c.max_acceleration = val;
    else if (reg == "servopos.position_min")        c.position_min    = val;
    else if (reg == "servopos.position_max")        c.position_max    = val;
    else if (reg == "servo.max_voltage")            c.max_voltage     = val;
    else if (reg == "servo.max_power_W")            c.max_power_W     = val;
    else if (reg == "servo.default_timeout_s")      c.def_timeout     = val;
}


// ---------------------------------------------------------------------------
// Calibration support
//
// Flow per motor:
//   1. Set calibrating_ = true  → poll() returns early, CAN bus goes idle
//   2. Wait 50 ms for any in-flight BlockingCycle to finish
//   3. Release transport_ and controllers_ so the CAN socket is closed
//   4. Run: python3 -m moteus.moteus_tool -t <id>
//              --calibrate --cal-motor-poles 16 --cal-force-kv 265 --cal-hal
//   5. Re-create transport_ + controllers_
//   6. conf set motor_position.sources.0.type type.hall:4
//   7. conf save
//   8. Re-configure all motors (re-push PID/limits)
//   9. Set calibrating_ = false
// ---------------------------------------------------------------------------

void MoteusDriverNode::publishCalibStatus(int motor_id, int state, const std::string& message) {
    rover_msgs::msg::MoteusCalibrationStatus s;
    s.motor_id = motor_id;
    s.state    = state;
    s.message  = message;
    calib_pub_->publish(s);
    publishLog("# calib motor " + std::to_string(motor_id) + ": " + message);
    RCLCPP_INFO(this->get_logger(), "[calib] motor %d: %s", motor_id, message.c_str());
}

// ---------------------------------------------------------------------------
// reInitTransport — re-create owned transport + controllers after calibration.
// Uses TransportRegistry (not singleton) so the new transport is fully owned.
// ---------------------------------------------------------------------------
void MoteusDriverNode::reInitTransport() {
    controllers_.clear();
    transport_.reset();  // fd is now closed — refcount hit 0

    transport_ = mot::TransportRegistry::singleton().make({}).first;

    for (int id = 1; id <= NUM_MOTORS; id++) {
        mot::Controller::Options opts;
        opts.id        = id;
        opts.transport = transport_;
        opts.query_format.q_current = mot::kFloat;
        opts.query_format.power     = mot::kFloat;
        opts.query_format.aux2_gpio = mot::kInt8;
        opts.position_format.velocity_limit = mot::kFloat;   
        opts.position_format.accel_limit    = mot::kFloat;  
        controllers_.push_back(std::make_shared<mot::Controller>(opts));
    }
}

// ---------------------------------------------------------------------------
// runCalibration — full Hall-sensor calibration workflow for one motor.
//
// Step 1  Pause poll loop (calibrating_ = true).
// Step 2  conf write — flush current RAM config to flash so our PID/limits
//         survive the calibration (moteus_tool may issue its own conf write).
// Step 3  Release transport — drops the only shared_ptr so the CAN fd closes.
//         This is safe because we used TransportRegistry::make() (not the
//         singleton), so we truly own the fd.
// Step 4  Run moteus_tool via popen with -u (unbuffered) so every output line
//         is streamed to /arm/config_log in real time.
// Step 5  Re-init transport + controllers.
// Step 6  conf set motor_position.sources.0.type type.hall:4
//         conf write — persist Hall sensor type to flash.
// Step 7  configureMotors() — re-push PID/limits into RAM.
// Step 8  Resume poll loop.
// ---------------------------------------------------------------------------
void MoteusDriverNode::runCalibration(int motor_can_id) {
    RCLCPP_ERROR(this->get_logger(), "Sorry, calibration is under construction... coming soon to a PR near you");
    // using namespace std::chrono_literals;

    // // ── Step 1: pause poll ────────────────────────────────────────────────────
    // calibrating_.store(true);
    // std::this_thread::sleep_for(60ms);  // let any in-flight BlockingCycle finish

    // // ── Step 2: conf write — persist current config before calibration ────────
    // // publishCalibStatus(motor_can_id, 1, "Saving config to flash before calibration...");
    // // publishLog("# calib motor " + std::to_string(motor_can_id) + ": conf write (pre-calib)");
    // // controllers_[motor_can_id - 1]->DiagnosticCommand("conf write");

    // // ── Step 3: release CAN fd ────────────────────────────────────────────────
    // publishCalibStatus(motor_can_id, 1, "Releasing CAN bus for moteus_tool...");
    // controllers_.clear();
    // transport_.reset();  // refcount → 0: CAN fd is now closed
    // std::this_thread::sleep_for(100ms);  // small grace period for OS to release the port

    // // ── Step 4: run moteus_tool, stream output line-by-line to command log ────
    // std::string cmd =
    //     "python3 -u -m moteus.moteus_tool"
    //     " -t " + std::to_string(motor_can_id) +
    //     " --calibrate"
    //     " --cal-motor-poles 16"
    //     " --cal-force-kv 265"
    //     " --cal-hal"
    //     " 2>&1";  // merge stderr so we see errors too

    // publishCalibStatus(motor_can_id, 1,
    //     "Running: " + cmd.substr(0, cmd.size() - 5));  // hide 2>&1 from HMI
    // publishLog("# calib motor " + std::to_string(motor_can_id) + ": " + cmd);

    // FILE* pipe = popen(cmd.c_str(), "r");
    // int exit_code = 1;
    // if (!pipe) {
    //     publishCalibStatus(motor_can_id, 3, "popen failed — cannot launch moteus_tool");
    // } else {
    //     char buf[512];
    //     while (fgets(buf, sizeof(buf), pipe)) {
    //         std::string line(buf);
    //         if (!line.empty() && line.back() == '\n') line.pop_back();
    //         publishLog("  [moteus_tool] " + line);
    //     }
    //     exit_code = pclose(pipe);
    //     // pclose returns the wait-status; extract real exit code
    //     if (WIFEXITED(exit_code)) exit_code = WEXITSTATUS(exit_code);
    // }

    // // ── Step 5: re-init transport ─────────────────────────────────────────────
    // reInitTransport();

    // if (exit_code != 0) {
    //     publishCalibStatus(motor_can_id, 3,
    //         "moteus_tool exited with code " + std::to_string(exit_code) + " — calibration FAILED");
    //     configureMotors();
    //     calibrating_.store(false);
    //     return;
    // }

    // // ── Step 6: set Hall sensor source and write to flash ─────────────────────
    // publishCalibStatus(motor_can_id, 1,
    //     "Setting motor_position.sources.0.type = type.hall:4 ...");
    // publishLog("# calib motor " + std::to_string(motor_can_id)
    //     + ": conf set motor_position.sources.0.type type.hall:4");
    // controllers_[motor_can_id - 1]->DiagnosticCommand(
    //     "conf set motor_position.sources.0.type type.hall:4");

    // // publishLog("# calib motor " + std::to_string(motor_can_id) + ": conf write (post-calib)");
    // // controllers_[motor_can_id - 1]->DiagnosticCommand("conf write");

    // // ── Step 7: re-push PID/limits into RAM ───────────────────────────────────
    // configureMotors();

    // publishCalibStatus(motor_can_id, 2,
    //     "Calibration complete.  Hall sensor type set and saved to flash.");

    // // ── Step 8: resume poll ───────────────────────────────────────────────────
    // calibrating_.store(false);
}

void MoteusDriverNode::calibrationCallback(
        const rover_msgs::msg::MoteusCalibrationRequest::SharedPtr msg)
{
    if (!calib_mutex_.try_lock()) {
        RCLCPP_WARN(this->get_logger(),
            "Calibration already in progress — request ignored.");
        return;
    }
    // mutex held; release it inside the thread after calibration finishes
    std::thread([this, motor_id = msg->motor_id]() {
        if (motor_id == 0) {
            // Sequential: calibrate all motors one by one
            for (int id = 1; id <= NUM_MOTORS; id++) {
                runCalibration(id);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        } else if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
            runCalibration(motor_id);
        } else {
            RCLCPP_WARN(this->get_logger(),
                "calibrationCallback: motor_id %d out of range", motor_id);
        }
        calib_mutex_.unlock();
    }).detach();
}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoteusDriverNode>());
    rclcpp::shutdown();
    return 0;
}
