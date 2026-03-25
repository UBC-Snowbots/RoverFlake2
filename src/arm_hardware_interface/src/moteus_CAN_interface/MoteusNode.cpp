#include "ArmMoteusInterface.h"

// This is the main node for interfacing with the moteus motor controllers over CAN-FD.
// It subscribes to /arm/command for incoming commands (from the joy node or MoveIt Servo) 
// and publishes feedback on /arm/feedback and /arm/moteus_feedback.

ArmCAN::ArmCAN() : Node("ArmCanDriver") {
  
  // Initialization
  for (int i = 0; i < NUM_JOINTS; i++) {
    axes[i].zero_rad = ArmConstants::axis_zero_rads[i];
    axes[i].dir = ArmConstants::axis_dirs[i];
    axes[i].index = i;
#ifdef PRINTOUT_AXIS_PARAMS
    RCLCPP_INFO(this->get_logger(), "Axis %i /// DIR[ %i ] /// OFFSET TO URDF's ZERO_RAD[ %f ] ", i + 1, axes[i].dir, axes[i].zero_rad);
#endif
  }

  // auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(); //Very hack way of only using "live" messages - iffy, and may still
  // operate off of one stale message. in the future we should use a time stamped message, and check the stamp time against current time to
  // make sure msg is not stale.
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

  arm_position_publisher = this->create_publisher<rover_msgs::msg::ArmCommand>("/arm/feedback", qos);           // arm position
  arm_status_publisher = this->create_publisher<rover_msgs::msg::MoteusArmStatus>("/arm/moteus_feedback", qos); // moteus driver status
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);          // joint states (pos/vel) -> moveit
  speaker_publisher = this->create_publisher<std_msgs::msg::String>("/speaker/command", qos);                   // speaker commands 
  double period = 1.0 / COMM_POLL_RATE;

  current_arm_status.positions.resize(NUM_JOINTS);

  command_subscriber = this->create_subscription<rover_msgs::msg::ArmCommand>(
      "/arm/command", 10, std::bind(&ArmCAN::CommandCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "moteus jointsff %d", NUM_JOINTS);
  if (!SIMULATE) {
    for (int i = 1; i <= NUM_JOINTS; i++) {
      moteus::Controller::Options options;
      options.id = i;
      controllers[i] = std::make_shared<moteus::Controller>(options);
    }
    
    // set period to 10ms (100Hz) to match firmware
    period = 1.0 / 100.0;
    transport = moteus::Controller::MakeSingletonTransport({});

    // Stop everything to clear faults and configure
    for (const auto &pair : controllers) {
      pair.second->SetStop();
      RCLCPP_INFO(this->get_logger(), "moteuss jointsff %d", NUM_JOINTS);
      ConfigureMotor(pair.first, *pair.second);
      RCLCPP_INFO(this->get_logger(), "moteuaas jointsff %d", NUM_JOINTS);
    }
    
    // setting the polling rate
    timer_ = this->create_wall_timer(std::chrono::duration<double>(period), std::bind(&ArmCAN::serial_rx, this));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  std_msgs::msg::String start_sound_msg;
  start_sound_msg.data = "AI_welcome_attention.wav";
  speaker_publisher->publish(start_sound_msg);
}