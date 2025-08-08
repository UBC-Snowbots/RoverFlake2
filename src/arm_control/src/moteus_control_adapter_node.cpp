#include "moteus_control_adapter_node.h"



// Constructor
 MoteusControlAdapter::MoteusControlAdapter() : Node("arm_moteus_control_adapter")
  {


    // cmd_sub_ = this->create_subscription<ArmCommand>(
    //     "/arm/command", rclcpp::SystemDefaultsQoS(),
    //     [this](const ArmCommand::SharedPtr msg){ onArmCommand(msg); }); // Fucking lambda gpt??
     
    command_subscriber = this->create_subscription<rover_msgs::msg::ArmCommand>(
      "/arm/command", 10, std::bind(&MoteusControlAdapter::CommandCallback, this, std::placeholders::_1));

    // Feedback source: prefer JointState from moteus bridge.

    // feedback_pub = this->create_publisher<ArmCommand>("/arm/feedback", rclcpp::SystemDefaultsQoS());

    // --- TODO(MOTEUS): set up publishers for moteus commands exactly as your stack expects.
    // Example patterns (commented to compile cleanly):
    // 1) Single aggregate command:
    // moteus_pub = this->create_publisher<moteus_msgs::msg::MultiVelocityCommand>("/moteus/velocity_command", 10);
    //
    // 2) Per-joint velocity command publishers:
    for (int i=1; i<=NUM_JOINTS; i++) {
      auto topic = "/moteus/id_" + std::to_string(i) + "/cmd_position";
      moteus_pubs.push_back(
         this->create_publisher<moteus_msgs::msg::PositionCommand>(topic, 10));
    }

    RCLCPP_INFO(get_logger(), "arm_moteus_adapter ready.");
  }


  





void MoteusControlAdapter::CommandCallback(const rover_msgs::msg::ArmCommand::SharedPtr rover_msg) {
  // deg/s -> turns/s
  for (size_t i = 0; i < NUM_JOINTS_NO_EE; ++i) {
    moteus_msgs::msg::PositionCommand cmd;
    cmd.position.resize(1);
    cmd.velocity.resize(1);
    cmd.accel_limit.resize(1);
    // position = NaN to select velocity-only behavior
    cmd.position[0] = {std::numeric_limits<double>::quiet_NaN()}; // idk i think we can just leave unset

    cmd.velocity[0] = rover_msg->velocities[i];   //
  
    cmd.accel_limit[0] = MoteusArmParams::max_accel[i];
    // (optional) accel/torque/timeout fields if you use them:
    // cmd.accel_limit = {5.0};
    // cmd.watchdog_timeout = {std::numeric_limits<double>::quiet_NaN()};
    moteus_pubs[i]->publish(cmd);
  }
  // Handle EE Seperatley
  moteus_msgs::msg::PositionCommand cmd;
      cmd.position.resize(1);
    cmd.velocity.resize(1);
    cmd.accel_limit.resize(1);
  cmd.velocity[0] = rover_msg->end_effector / 10;
  cmd.position[0] = {std::numeric_limits<double>::quiet_NaN()};
  cmd.accel_limit[0] = MoteusArmParams::max_accel[EE_INDEX];
  moteus_pubs[EE_INDEX]->publish(cmd);

  
}


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoteusControlAdapter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
