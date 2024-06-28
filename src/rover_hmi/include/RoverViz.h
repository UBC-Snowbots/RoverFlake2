#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/arm_command.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"


namespace rvt = rviz_visual_tools;

//?demo https://github.com/PickNikRobotics/rviz_visual_tools/blob/ros2/src/rviz_visual_tools_demo.cpp

class RoverViz : public rclcpp::Node {
public:
    RoverViz() : Node("middle_screen_node") {
        RCLCPP_INFO(this->get_logger(), "RoverViz Start");


        // initializeVisualTools();

    // Allow time for RViz to start
    rclcpp::sleep_for(std::chrono::seconds(3));
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_frame","/rviz_visual_markers", this));
    //   visual_tools_->loadMarkerPub();
    //       bool has_sub = visual_tools_->waitForMarkerSub(10.0);
    // if (!has_sub)
    //   RCLCPP_INFO(get_logger(), "/rviz_visual_tools does not have a subscriber after 10s. "
    //                             "Visualizations may be lost");
    cam_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    // // Clear messages
    visual_tools_->deleteAllMarkers();
    visual_tools_->enableBatchPublishing();
    // Define the position for the label
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().x() = 1;  // Adjust the x position
    text_pose.translation().y() = 1;  // Adjust the y position
    // text_pose.translation().z() = 1;  // Adjust the z position

    // Publish the text label

//    std::string labelmeow = "meow";
    visual_tools_->publishText(text_pose, "labelmeow", rvt::WHITE, rvt::XXLARGE, false);
    // Trigger the visual tools to actually display the text
    visual_tools_->trigger();

        camera_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("tf", 10);
        camera_pose_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RoverViz::update_camera_pose, this));
    }

    ~RoverViz(){
        RCLCPP_INFO(this->get_logger(), "RoverViz Stop");
    }


private:
    // void initializeVisualTools() {
    //     visual_tools_ = std::make_shared<rviz_visual_tools::RvizVisualTools>(
    //         "base_link", "/rviz_visual_markers", shared_from_this());
    // }
    void armFeedbackCallback(const rover_msgs::msg::ArmCommand::SharedPtr msg);
    void update_camera_pose();

    rvt::RvizVisualToolsPtr visual_tools_;
        rclcpp::TimerBase::SharedPtr camera_pose_timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr camera_publisher_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> cam_tf_broadcaster_;
};
