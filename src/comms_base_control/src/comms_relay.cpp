#include "comms_relay.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

using ServoCommand = rover_msgs::msg::ServoCommand;
using Trigger = std_srvs::srv::Trigger;

namespace {

// ServoCommand still selects servos by number; these are their config names.
const char* servoCommandName(uint8_t servo) {
    switch (servo) {
        case ServoCommand::SERVO_RP:
            return "rp";
        case ServoCommand::SERVO_CLAW:
            return "claw";
        default:
            return nullptr;
    }
}

// Reads config/servo_types/<name>.yaml; throws naming the file and field.
ServoType loadServoType(const std::string& dir, const std::string& name) {
    const std::string path = dir + "/" + name + ".yaml";
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        // yaml-cpp's message already names the file.
        throw std::runtime_error(std::string("cannot read servo type: ") + e.what());
    }
    auto field = [&](const char* key) {
        if (!root[key]) {
            throw std::runtime_error(path + ": missing required field '" + key + "'");
        }
        try {
            return root[key].as<float>();
        } catch (const YAML::Exception&) {
            throw std::runtime_error(path + ": field '" + key + "' is not a number");
        }
    };
    ServoType type;
    type.name = name;
    type.min_us = field("min_us");
    type.max_us = field("max_us");
    type.open_us = field("open_us");
    type.close_us = field("close_us");
    return type;
}

// YAML gives 1600 as an int and 1600.0 as a double; accept either.
float numberParam(const rclcpp::Parameter& param) {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        return static_cast<float>(param.as_int());
    }
    return static_cast<float>(param.as_double());
}

const std::set<std::string> SHUTDOWN_ACTIONS = {"min", "max", "open", "close", "release"};

}  // namespace

ServoControlNode::ServoControlNode()
    : Node("servo_control_node",
           rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    default_hold_ms_ = this->get_parameter_or<int>("default_hold_ms", 1000);
    const std::string servo_types_dir = this->get_parameter_or<std::string>(
        "servo_types_dir",
        ament_index_cpp::get_package_share_directory("comms_base_control") +
            "/config/servo_types");

    // Connect to an already-running pigpiod rather than driving the hardware
    // directly, so this node does not need root. Passing nulls means
    // localhost:8888 unless PIGPIO_ADDR / PIGPIO_PORT say otherwise.
    pi_ = pigpio_start(nullptr, nullptr);
    if (pi_ < 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to connect to pigpio daemon (%d); is pigpiod running?", pi_);
    } else {
        RCLCPP_INFO(this->get_logger(), "Connected to pigpio daemon (handle %d)", pi_);
    }

    try {
        loadServos(servo_types_dir);
    } catch (...) {
        // The destructor won't run for a half-built node, so close the
        // pigpiod connection here before reporting the bad config.
        if (pi_ >= 0) {
            pigpio_stop(pi_);
        }
        throw;
    }
    for (auto& [name, entry] : servos_) {
        entry.servo.close();
    }

    // Subscribed even when pigpiod is unreachable, so commands are logged as
    // rejected instead of vanishing into a topic nobody listens on.
    servo_command_sub_ = this->create_subscription<ServoCommand>(
        "/servo_control_node/servo_command", 10,
        std::bind(&ServoControlNode::handleServoCommand, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "ServoControlNode initialization complete with %zu servo(s), all closed.",
                servos_.size());
}

ServoControlNode::~ServoControlNode() {
    if (pi_ < 0) {
        return;
    }
    for (auto& [name, entry] : servos_) {
        Servo& servo = entry.servo;
        if (entry.on_shutdown == "min") {
            servo.goToMin();
        } else if (entry.on_shutdown == "max") {
            servo.goToMax();
        } else if (entry.on_shutdown == "open") {
            servo.open();
        } else if (entry.on_shutdown == "close") {
            servo.close();
        } else {
            servo.release();
        }
    }
    pigpio_stop(pi_);
}

void ServoControlNode::loadServos(const std::string& servo_types_dir) {
    // Servo names are the <name> in parameters named servos.<name>.<field>.
    const std::string prefix = "servos.";
    std::set<std::string> names;
    for (const auto& param : this->list_parameters({}, 10).names) {
        if (param.rfind(prefix, 0) == 0) {
            const auto sub = param.substr(prefix.size());
            const auto dot_pos = sub.find('.');
            if (dot_pos != std::string::npos) {
                names.insert(sub.substr(0, dot_pos));
            }
        }
    }
    if (names.empty()) {
        throw std::runtime_error("no servos configured; expected servos.<name>.type parameters");
    }

    for (const auto& name : names) {
        const std::string p = prefix + name + ".";
        if (!this->has_parameter(p + "type") || !this->has_parameter(p + "gpio_pin")) {
            throw std::runtime_error("servo '" + name + "' needs both 'type' and 'gpio_pin'");
        }
        ServoType type =
            loadServoType(servo_types_dir, this->get_parameter(p + "type").as_string());
        // Per-servo overrides, e.g. a mechanism that allows less travel than
        // the servo model itself.
        for (auto [key, field] : {std::pair{"min_us", &ServoType::min_us},
                                  std::pair{"max_us", &ServoType::max_us},
                                  std::pair{"open_us", &ServoType::open_us},
                                  std::pair{"close_us", &ServoType::close_us}}) {
            if (this->has_parameter(p + key)) {
                type.*field = numberParam(this->get_parameter(p + key));
            }
        }
        try {
            type.validate();
        } catch (const std::runtime_error& e) {
            throw std::runtime_error("servo '" + name + "': " + e.what());
        }

        const auto on_shutdown = this->get_parameter_or<std::string>(p + "on_shutdown", "release");
        if (SHUTDOWN_ACTIONS.count(on_shutdown) == 0) {
            throw std::runtime_error("servo '" + name + "': unknown on_shutdown '" + on_shutdown +
                                     "' (expected min|max|open|close|release)");
        }
        const auto gpio_pin = static_cast<int>(this->get_parameter(p + "gpio_pin").as_int());
        servos_.emplace(name, ServoEntry{Servo(name, gpio_pin, type, pi_), on_shutdown});

        for (bool open : {true, false}) {
            open_close_srvs_.push_back(this->create_service<Trigger>(
                "~/" + name + (open ? "/open" : "/close"),
                [this, name, open](const std::shared_ptr<Trigger::Request>,
                                   std::shared_ptr<Trigger::Response> response) {
                    handleOpenClose(name, open, *response);
                }));
        }

        RCLCPP_INFO(this->get_logger(),
                    "%s servo: type %s on GPIO %d (range %.0f-%.0f us, open %.0f, close %.0f)",
                    name.c_str(), type.name.c_str(), gpio_pin, type.min_us, type.max_us,
                    type.open_us, type.close_us);
    }
}

void ServoControlNode::handleOpenClose(const std::string& name, bool open,
                                       Trigger::Response& response) {
    Servo& servo = servos_.at(name).servo;

    // Open/close hold their position, so drop any pending return to rest.
    auto timer = return_timers_.find(name);
    if (timer != return_timers_.end()) {
        timer->second->cancel();
    }

    response.success = open ? servo.open() : servo.close();
    if (response.success) {
        response.message = name + (open ? " open (" : " closed (") +
                           std::to_string(std::lround(servo.pulseUs())) + " us)";
        RCLCPP_INFO(this->get_logger(), "%s", response.message.c_str());
    } else {
        response.message = name + ": failed to " + (open ? "open" : "close") +
                           " (not connected to pigpio daemon?)";
        RCLCPP_ERROR(this->get_logger(), "%s", response.message.c_str());
    }
}

void ServoControlNode::handleServoCommand(const ServoCommand::SharedPtr msg) {
    if (pi_ < 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "servo_command ignored: not connected to pigpio daemon");
        return;
    }

    const char* name = servoCommandName(msg->servo);
    auto it = name ? servos_.find(name) : servos_.end();
    if (it == servos_.end()) {
        RCLCPP_ERROR(this->get_logger(),
                     "servo_command ignored: unknown servo %u (expected 0=rp or 1=claw, "
                     "configured in servos.yaml)",
                     msg->servo);
        return;
    }
    Servo& servo = it->second.servo;
    const ServoType& type = servo.type();

    float pwm;
    switch (msg->mode) {
        case ServoCommand::MODE_LIMIT_MIN:
            pwm = type.min_us;
            break;
        case ServoCommand::MODE_LIMIT_MAX:
            pwm = type.max_us;
            break;
        case ServoCommand::MODE_PWM:
            if (!std::isfinite(msg->pwm_us)) {
                RCLCPP_ERROR(this->get_logger(),
                             "servo_command ignored: requested pulse width is not a finite number");
                return;
            }
            // Clamp rather than reject, so a slightly out-of-range command still
            // moves the servo to the nearest safe endpoint instead of doing nothing.
            pwm = std::clamp(msg->pwm_us, type.min_us, type.max_us);
            if (pwm != msg->pwm_us) {
                RCLCPP_WARN(this->get_logger(), "%s servo: %.0f us clamped to %.0f us", name,
                            msg->pwm_us, pwm);
            }
            break;
        default:
            RCLCPP_ERROR(this->get_logger(),
                         "servo_command ignored: unknown mode %u (expected 0=MIN, 1=MAX, 2=PWM)",
                         msg->mode);
            return;
    }

    const uint16_t hold_ms =
        msg->hold_ms != 0 ? msg->hold_ms : static_cast<uint16_t>(default_hold_ms_);

    if (!servo.setPulseUs(pwm)) {
        RCLCPP_ERROR(this->get_logger(), "%s servo: pigpio daemon refused %.0f us", name, pwm);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "%s servo -> %.0f us (holding %u ms)", name, pwm, hold_ms);

    scheduleReturnToRest(name, hold_ms);
}

void ServoControlNode::scheduleReturnToRest(const std::string& name, uint16_t hold_ms) {
    // Assigning over the entry cancels whatever hold was already pending for
    // this servo, so the newest command owns the return.
    return_timers_[name] = this->create_wall_timer(
        std::chrono::milliseconds(hold_ms), [this, name]() {
            // One-shot: stop the timer before doing the work so it cannot repeat.
            return_timers_[name]->cancel();
            Servo& servo = servos_.at(name).servo;
            // Rest is the closed position, the same state the node starts in.
            servo.close();
            RCLCPP_INFO(this->get_logger(), "%s servo returned to rest (%.0f us)", name.c_str(),
                        servo.pulseUs());
        });
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<ServoControlNode> node;
    try {
        node = std::make_shared<ServoControlNode>();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("servo_control_node"), "Bad servo configuration: %s",
                     e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
