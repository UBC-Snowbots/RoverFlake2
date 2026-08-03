//* Per-computer supervisor: starts/stops the subsystems listed in heart.yaml and
//* reports their OBSERVED state (reaped via waitpid, never a stored belief).
#include <rclcpp/rclcpp.hpp>
#include <rover_utils/include/roverCommon.h>
#include <rover_msgs/msg/subsystem_command.hpp>
#include <rover_msgs/msg/heart_status.hpp>
#include <chrono>
#include <sys/types.h>

class HeartNode : public rclcpp::Node
{
public:
    HeartNode();

private:
    using State = rover_msgs::msg::SubsystemState;
    struct SubSystem {
        std::string name;
        std::string exec_command;
        pid_t pid = -1;              // == pgid/sid (child calls setsid)
        uint8_t state = State::STOPPED;
        int exit_code = 0;
        bool pending_restart = false;
        int stop_stage = 0;          // 1=SIGINT sent, 2=SIGTERM, 3=SIGKILL
        std::chrono::steady_clock::time_point started_at, escalate_at;
    };
    std::map<std::string, SubSystem> subsystems;  // ordered -> stable msg order
    std::string my_host_id;

    void startSubsystem(SubSystem& s);
    void stopSubsystem(SubSystem& s);
    void superviseTick();            // 250 ms: reap children, escalate stops
    void publishStatus();
    void commandCallback(rover_msgs::msg::SubsystemCommand::SharedPtr cmd);
    void signalGroup(pid_t pgid, int sig);

    rclcpp::TimerBase::SharedPtr heartbeat_timer, supervise_timer;
    rclcpp::Subscription<rover_msgs::msg::SubsystemCommand>::SharedPtr command_sub;
    rclcpp::Publisher<rover_msgs::msg::HeartStatus>::SharedPtr status_pub;
};
