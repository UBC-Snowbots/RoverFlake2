// TODO: make a good comment
#include "ArmSerialInterface.h"

void ArmSerial::CommandCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg) {
  char type = msg->cmd_type;
  switch (type) {
  case HOME_CMD:
    if (!SIMULATE) {
      sendHomeCmd(msg->cmd_value);
    }
    break;
  case COMM_CMD:
    if (!SIMULATE) {
      sendCommCmd(msg->cmd_value);
    }
    break;
  case TEST_LIMITS_CMD:
    if (!SIMULATE) {
      send_test_limits_command();
    }
    break;
  case ABS_POS_CMD:
    float target_positions[NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS; i++) {
      target_positions[i] = msg->positions[i];
    }
    if (SIMULATE) {
      sensor_msgs::msg::JointState joint_states_;
      joint_states_.position.resize(NUM_JOINTS);
      joint_states_.name.resize(NUM_JOINTS);
      for (int i = 0; i < NUM_JOINTS; i++) {
        joint_states_.name[i] = joint_names[i];
        joint_states_.position[i] = firmToMoveitOffsetPos(target_positions[i], i);
        joint_states_.velocity[i] = firmToMoveitOffsetVel(current_velocity[i], i);
      }
      joint_states_.header.stamp = rclcpp::Clock().now();

      joint_state_publisher_->publish(joint_states_);

    } else {
      send_position_command(target_positions);
    }
    break;
  case ABS_VEL_CMD:
    float target_velocities[NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS; i++) {
      target_velocities[i] = msg->velocities[i];
      current_velocity[i] = msg->velocities[i];
    }
    if (SIMULATE) {
      sensor_msgs::msg::JointState joint_states_;
      joint_states_.velocity.resize(NUM_JOINTS);
      joint_states_.name.resize(NUM_JOINTS);
      for (int i = 0; i < NUM_JOINTS; i++) {
        joint_states_.name[i] = joint_names[i];
        joint_states_.velocity[i] = firmToMoveitOffsetVel(target_velocities[i], i);
      }
      joint_states_.header.stamp = rclcpp::Clock().now();

      joint_state_publisher_->publish(joint_states_);

    } else {
      send_velocity_command(target_velocities);
    }
    break;
  default:
    break;
  }
}

ArmSerial::ArmSerial() : Node("ArmSerialDriver") {
  //? new arm offsets
  //? Axis 1
  //? -0.68 -> from online app thing
  //?  0.2808234691619873 -> read in
  axes[0].zero_rad = -0.9608; //? pree good
  axes[0].dir = 1;

  //? Axis 2
  //? -1.01   ISH - fack
  //? 0.9290387630462646
  axes[1].zero_rad = -1.9390; //? ISH
  axes[1].dir = 1;

  //? Axis 3
  //? -0.60 from online app
  //? 0.7459537386894226
  axes[2].zero_rad = -1.3460;
  axes[2].dir = 1;

  //? Axis 4
  //? 0.037 from online app
  //? 2.447824239730835
  axes[3].zero_rad = -2.4108; //? gear reduction probably wrong
  axes[3].dir = -1;

  //? Axis 5
  //? -0.62 from online app
  //? 1.585980772972107
  axes[4].zero_rad = -2.2060;
  axes[4].dir = 1;

  //? Axis 6
  axes[5].zero_rad = 0.0;
  axes[5].dir = 1;
  //?old arm offsets
  // axes[0].zero_rad = 0.984;
  // axes[0].dir = -1;

  // axes[1].zero_rad = 1.409;
  // axes[1].dir = -1;

  // axes[2].zero_rad = -0.696;
  // axes[2].dir = 1;

  // axes[3].zero_rad = 1.8067995;
  // axes[3].dir = -1;

  // axes[4].zero_rad = -1.002;
  // axes[4].dir = 1;

  // axes[5].zero_rad = -1.375;
  // axes[5].dir = 1;

  auto qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(); // Very hack way of only using "live" messages - iffy, and may still operate off
                                                          // of one stale message. in the future we should use a time stamped message, and
                                                          // check the stamp time against current time to make sure msg is not stale.
  // command_publisher_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd",
  // qos); gear_publisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", qos);
  arm_position_publisher = this->create_publisher<rover_msgs::msg::ArmCommand>("/arm/feedback", qos);
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);
  command_subscriber = this->create_subscription<rover_msgs::msg::ArmCommand>(
      "/arm/command", 10, std::bind(&ArmSerial::CommandCallback, this, std::placeholders::_1));

  current_arm_status.positions.resize(NUM_JOINTS);

  // joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
  //     "/joy", 10, std::bind(&ArmSerial::joy_callback, this, std::placeholders::_1));
  if (!SIMULATE) {
    setupSerial();
  }
  //  teensy.setDTR(true);
  //  teensy.setRTS(false);
  sleep(0.1);

  // int flag = 1;
  // while(rclcpp::ok()){
  //     recieveMsg();
  // }
  // set serial rx on a quick polling timer
}

// void ArmSerial::recieveMsg() {}

float ArmSerial::degToRad(float deg) {
  float rad = deg * 3.14159 / 180; // 14159265359

  return (rad);
}

float ArmSerial::firmToMoveitOffsetPos(float deg, int i) {

  float rad = degToRad(deg);

  return ((rad * axes[i].dir) + (axes[i].zero_rad));
}

float ArmSerial::firmToMoveitOffsetVel(float deg, int i) {

  float rad = degToRad(deg);

  return ((rad * axes[i].dir));
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmSerial>();

  RCLCPP_INFO(node->get_logger(), "ArmSerial init");
  // std::thread arm_thread(ArmSerial::SerialRxThread, std::ref(node));

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
