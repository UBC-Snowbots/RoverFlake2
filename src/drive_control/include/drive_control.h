#include <phidget22.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.h>
// #include <rover_utils/rover_utils.h>

//* https://www.phidgets.com/?view=api

#define VERBOSE
class DriveControlNode : public rclcpp::Node
{
public:
    DriveControlNode() : Node("drive_control_node") //Constructor. set up publishers and subscribers here
    {
        #ifdef VERBOSE
            RCLCPP_INFO(this->get_logger(), "Drive Control Initiated");
        #endif

        PhidgetReturnCode ret;

        for(int i = 0; i < NUM_MOTORS; i++){
            PhidgetBLDCMotor_create(&motors[i]);

        }

    }

    ~DriveControlNode(){
        #ifdef VERBOSE
            RCLCPP_WARN(this->get_logger(), "Drive Control Offline");
        #endif 
    }

private:

        const int static NUM_MOTORS = 6;
    PhidgetBLDCMotorHandle bldcMotor0, bldcMotor1, bldcMotor2, bldcMotor3,
    bldcMotor4, bldcMotor5;
    std::vector<PhidgetBLDCMotorHandle> motors{
    bldcMotor0, bldcMotor1, bldcMotor2, bldcMotor3, bldcMotor4, bldcMotor5};
};