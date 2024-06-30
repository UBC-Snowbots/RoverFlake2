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
    double position = 0.0;
        // PhidgetReturnCode ret;
    for (int i = 0; i < NUM_MOTORS; i++) {

        PhidgetBLDCMotor_create(&motors[i]);
        ret = Phidget_setHubPort((PhidgetHandle) motors[i], i);
        if (ret != EPHIDGET_OK) {
            Phidget_getLastError(
            &errorCode, &errorString, errorDetail, &errorDetailLen);
            RCLCPP_ERROR(this->get_logger(), "Error at set hub (%d) for port %d: %s", errorCode, i, errorString);
            return;
        }else{
            RCLCPP_INFO(this->get_logger(), "hub attached succesfully at %d", i);
         //   pub_vitals(STANDBY);
        }
        ret = Phidget_openWaitForAttachment((PhidgetHandle) motors[i], 5000);
        if (ret != EPHIDGET_OK) {
            Phidget_getLastError(
            &errorCode, &errorString, errorDetail, &errorDetailLen);
            RCLCPP_ERROR(this->get_logger(), "Error at attachment (%d) for port %d:, %s ",
                      errorCode,
                      i,
                      errorString);
          //  pub_vitals(ERROR);
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Attached successfully for port %d", i);

        }
    }
        // for(int i = 0; i < NUM_MOTORS; i++){
        //     PhidgetBLDCMotor_create(&motors[i]);

        // }
        
        PhidgetBLDCMotor_setTargetVelocity((PhidgetBLDCMotorHandle) motors[0], 0.5);
        PhidgetBLDCMotor_enableFailsafe((PhidgetBLDCMotorHandle) motors[0], 612);
        PhidgetBLDCMotor_getPosition(motors[0], &position);
        RCLCPP_INFO(this->get_logger(), "Position: %f", position);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DriveControlNode::check_motor_positions, this)
        );
        
    }

    ~DriveControlNode(){
        #ifdef VERBOSE
            RCLCPP_WARN(this->get_logger(), "Drive Control Offline");
        #endif 
    }
void check_motor_positions()
{
    double position = 0.0;
    double scaled_position = 0.0;
    for (int i = 0; i < NUM_MOTORS; i++) {
        PhidgetBLDCMotor_getPosition(motors[i], &position);
        scaled_position = position *1.3666;
        RCLCPP_INFO(this->get_logger(), "Motor %d Position: %f", i, scaled_position);
    }
}

private:

    const int static NUM_MOTORS = 1;

    PhidgetBLDCMotorHandle bldcMotor0, bldcMotor1, bldcMotor2, bldcMotor3,
    bldcMotor4, bldcMotor5;

    std::vector<PhidgetBLDCMotorHandle> motors{
    bldcMotor0, bldcMotor1, bldcMotor2, bldcMotor3, bldcMotor4, bldcMotor5};
    const std::vector<int> right_motors{0, 1, 2};
    const std::vector<int> left_motors{3, 4, 5};
    PhidgetReturnCode ret;
    PhidgetReturnCode errorCode;
    PhidgetReturnCode res;
    const char* errorString;
    char errorDetail[100];
    size_t errorDetailLen = 100;
    rclcpp::TimerBase::SharedPtr timer_;

};