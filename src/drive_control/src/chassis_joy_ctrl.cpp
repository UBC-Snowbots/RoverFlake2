#include <chassis_joy_ctrl.h>

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChassisJoyCtrl>();
    rclcpp::spin(node);
    rclcpp::shutdown(); 
    return 0;
}

void ChassisJoyCtrl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg){
    geometry_msgs::msg::Twist out_msg;

    out_msg.linear.x = msg->axes[1] *30;
    out_msg.angular.z = msg->axes[3] *6;
    cmd_vel_pub->publish(out_msg);
}