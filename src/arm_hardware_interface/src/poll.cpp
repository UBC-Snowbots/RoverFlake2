#include "moteus_poll.h"

MoteusPoll::MoteusPoll() : Node("moteus_poll") {
    transport_ = moteus::Controller::MakeSingletonTransport({});

    for (int id = 1; id <= NUM_JOINTS; id++) {
        moteus::Controller::Options opts;
        opts.id = id;
        auto controller = std::make_shared<moteus::Controller>(opts);
        controllers_.push_back(controller);
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MoteusPoll::poll, this)
    );

    RCLCPP_INFO(this->get_logger(), "Polling %d motors at 10 Hz", NUM_JOINTS);
}

void MoteusPoll::poll() {
    std::vector<moteus::CanFdFrame> command_frames;

    for (auto& controller : controllers_) {
        command_frames.push_back(controller->MakeQuery());
    }

    std::vector<moteus::CanFdFrame> replies;
    transport_->BlockingCycle(command_frames.data(), command_frames.size(), &replies);

    for (const auto& frame : replies) {
        int motor_id = frame.source;
        auto result = moteus::Query::Parse(frame.data, frame.size);

        RCLCPP_INFO(this->get_logger(),
            "ID: %d | Mode: %2d | Fault: %2d | "
            "Pos: %.3f rev | Vel: %.3f rev/s | Torque: %.3f Nm | "
            "Voltage: %.1f V | Temp: %.1f C",
            motor_id,
            static_cast<int>(result.mode),
            static_cast<int>(result.fault),
            result.position,
            result.velocity,
            result.torque,
            result.voltage,
            result.temperature
        );
    }

    RCLCPP_INFO(this->get_logger(), "---");
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoteusPoll>());
    rclcpp::shutdown();
    return 0;
}
