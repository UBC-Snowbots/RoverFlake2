// AlertRaiser: Check out alert_example.cpp in rover_utils for usage
// This is to standardize any alerts and track them. It needs a parent node.
// Alerts are stored with their addresses as keys. so all must be allocated once as member variables.
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/active_alert.hpp>
#include <rover_msgs/msg/ack_alert.hpp>
#include <rover_utils/config/topicNames.h>
#include <rover_utils/include/time_utils.h>
#include <rover_utils/include/fancyOutput.h>
#include <chrono>

#define MANAGE_ALERTS_CYCLE_PERIOD_S 2.0

// #define ALERT_DEBUG
using namespace ConsoleFormat;
using namespace std::chrono_literals;
using namespace std::chrono;

static auto MAX_UNACKED_DURATION = 3000ms;
static auto MIN_REPUBLISH_PERIOD = 2000ms;

enum class AlertSeverity : uint8_t {
    abort = 1,
    warn = 2,
    info = 3,
    special = 4 // Reserved
};

// User-facing. Create these as member variables.
struct Alert {
    AlertSeverity severity;
    std::string message;
    // std::string mitigation = "NO_MIT"; //? maybe add a mitigation message?
    //? Should we put in a needs_acking flag?
    
    // Safegaurds against copy operators (if alert is copied, this class will store the wrong address, leading to undefined behaviour)
    Alert() = default;
    Alert(AlertSeverity s) : severity(s) {}

    // Force auto& in loops
__attribute__((error(
    "\033[34m\033[1m"  // blue + bold
    "Alert must not be copied!"
    "\033[0m"          // reset
    " Use "
    "\033[32m"         // green
    "auto&"
    "\033[0m"          // reset
    " in loops, not "
    "\033[31m"         // red
    "auto"
    "\033[0m"          // reset
    ". You silly goose!"

)))
Alert(const Alert&);

__attribute__((error(
    "\033[35m\033[1m"
    "Alert must not be copied!"
    "\033[0m"
    " Use "
    "\033[32m"
    "auto&"
    "\033[0m"
    " in loops, not "
    "\033[31m"
    "auto"
    "\033[0m"
    ". You silly goose!"

)))
Alert& operator=(const Alert&);
};

class AlertRaiser
{
public:
    AlertRaiser(rclcpp::Node* node)
    {
        parent_node = node;
        #ifdef ALERT_DEBUG
        RCLCPP_INFO(parent_node->get_logger(), "Alert Raiser Start Init");
        #endif
        std::cout << ConsoleFormat::magenta() << "AlertRaiser" << ConsoleFormat::reset()
                  << " attached to node: " << ConsoleFormat::bright_magenta() << ConsoleFormat::bold()
                  << parent_node->get_fully_qualified_name() << reset() << std::endl;

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

        alert_pub = parent_node->create_publisher<rover_msgs::msg::ActiveAlert>(
            TOPIC_NAME_ALERTS_ACTIVE, qos);

        alert_sub = parent_node->create_subscription<rover_msgs::msg::AckAlert>(
            TOPIC_NAME_ALERTS_ACKS, qos,
            std::bind(&AlertRaiser::ackCallback, this, std::placeholders::_1));

        timer = parent_node->create_wall_timer(
            std::chrono::duration<double>(MANAGE_ALERTS_CYCLE_PERIOD_S),
            std::bind(&AlertRaiser::timer_callback, this));
    }

    void raise(Alert& alert);
    void update(Alert& alert);
    void clear(Alert& alert);
    bool isActive(const Alert& alert) const;

private:
    rclcpp::Node* parent_node;

    enum class AckStatus : uint8_t {
        unacked,
        displayed,
        cleared
    };

    struct TrackedAlert {
        Alert* alert_ptr;
        uint32_t uid;
        steady_clock::time_point raised_at;
        steady_clock::time_point last_published;
        steady_clock::time_point last_acked;
    };

    TrackedAlert* findByUid(uint32_t uid);
    void ackCallback(const rover_msgs::msg::AckAlert::SharedPtr msg);
    void timer_callback();
    void publish(const TrackedAlert& tracked);

    uint32_t uid_counter = 0;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<rover_msgs::msg::ActiveAlert>::SharedPtr alert_pub;
    rclcpp::Subscription<rover_msgs::msg::AckAlert>::SharedPtr alert_sub;

    std::unordered_map<const Alert*, TrackedAlert> active_alerts;
};

bool AlertRaiser::isActive(const Alert& alert) const
{
    return active_alerts.count(&alert) > 0;
}

AlertRaiser::TrackedAlert* AlertRaiser::findByUid(uint32_t uid)
{
    for (auto& [ptr, tracked] : active_alerts) {
        if (tracked.uid == uid)
            return &tracked;
    }
    return nullptr;
}

void AlertRaiser::raise(Alert& alert)
{
    if (isActive(alert))
        return;

    TrackedAlert tracked;
    tracked.alert_ptr = &alert;
    tracked.uid = ++uid_counter;
    tracked.raised_at = steady_clock::now();
    tracked.last_published = tracked.raised_at;
    tracked.last_acked = steady_clock::time_point{};

    active_alerts[&alert] = tracked;

    #ifdef ALERT_DEBUG
    RCLCPP_INFO(parent_node->get_logger(), "Alert raised uid=%u", tracked.uid);
    #endif

    publish(active_alerts[&alert]);
}

void AlertRaiser::update(Alert& alert)
{
    if (!isActive(alert)) {
        raise(alert);
        return;
    }

    auto& tracked = active_alerts[&alert];

    #ifdef ALERT_DEBUG
    RCLCPP_INFO(parent_node->get_logger(), "Alert updated uid=%u severity=%u",
        tracked.uid, static_cast<uint8_t>(alert.severity));
    #endif

    publish(tracked);
    tracked.last_published = steady_clock::now();
}

void AlertRaiser::clear(Alert& alert)
{
    if (!isActive(alert))
        return;

    #ifdef ALERT_DEBUG
    RCLCPP_INFO(parent_node->get_logger(), "Alert cleared uid=%u", active_alerts[&alert].uid);
    #endif

    active_alerts.erase(&alert);
}

void AlertRaiser::publish(const TrackedAlert& tracked)
{
    rover_msgs::msg::ActiveAlert msg;
    msg.header = rover_utils::createHeader();
    msg.info_msg = tracked.alert_ptr->message;
    msg.severity = static_cast<uint8_t>(tracked.alert_ptr->severity);
    msg.nuid = tracked.uid;
    msg.source_node_name = parent_node->get_fully_qualified_name();
    alert_pub->publish(msg);
}

void AlertRaiser::timer_callback()
{
    for (auto& [ptr, tracked] : active_alerts) {
        if (tracked.alert_ptr->severity != AlertSeverity::abort
            && tracked.alert_ptr->severity != AlertSeverity::warn)
        {
            continue;
        }
        if (steady_clock::now() - tracked.last_acked > MAX_UNACKED_DURATION
            && steady_clock::now() - tracked.last_published > MIN_REPUBLISH_PERIOD)
        {
            publish(tracked);
            tracked.last_published = steady_clock::now();
        }
    }

    #ifdef ALERT_DEBUG
    RCLCPP_INFO(parent_node->get_logger(), "Alert Timer Hit");
    #endif
}

// Acking (acknoleging) is how alerts are cleared. This stops the alert from being raised, and allows it to be raised again. 
// Acking display is currently optional. Without acking, the alert just keeps getting republished. Acking display is letting this alert know that its displayed to operators.
void AlertRaiser::ackCallback(const rover_msgs::msg::AckAlert::SharedPtr msg)
{
    if (msg->target_node_name != parent_node->get_fully_qualified_name())
        return;

    TrackedAlert* tracked = findByUid(msg->nuid);
    if (!tracked)
        return;

    switch (static_cast<AckStatus>(msg->ack))
    {
        case AckStatus::cleared:
            active_alerts.erase(tracked->alert_ptr);
            break;
        case AckStatus::displayed:
            tracked->last_acked = steady_clock::now();
            break;
        default:
            break;
    }
}