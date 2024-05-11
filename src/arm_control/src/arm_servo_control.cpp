#include "armServoControl.h"


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmMoveitControl>();

    // Example: send a command with a steering angle of 0.5 rad and speed of 1.0 m/s

    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}

float ArmServoControl::radToDeg(float rad){
  float deg = (rad *180.0) / 3.14159265359;

  return(deg);
}

float ArmServoControl::moveitToFirmwareOffset(float rad, int i){

float deg;

  deg = (rad - axes[i].zero_rad)*axes[i].dir;
    
deg = radToDeg(deg);

return (deg);


}
