#include <speaker_node.h>

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpeakerNode>();
 node->init();
//   rclcpp::Rate hud_refresh_rate(40);
//   while (rclcpp::ok()){
//     if(node->hud.initiated){
//       RCLCPP_ERROR(node->get_logger(), "frame success");
//     node->drawHud();

//     }else{
//       RCLCPP_ERROR(node->get_logger(), "empty frame");

//     }
//     rclcpp::spin_some(node);
//     hud_refresh_rate.sleep();
//   }
rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
