#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/arm_command.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#define UPDATE_INTERVAL_SECONDS 0.5

namespace rvt = rviz_visual_tools;

//? demo https://github.com/PickNikRobotics/rviz_visual_tools/blob/ros2/src/rviz_visual_tools_demo.cpp
//? documentation http://docs.ros.org/en/melodic/api/rviz_visual_tools/html/namespacerviz__visual__tools.html

class RoverViz : public rclcpp::Node {
public:
    RoverViz() : Node("middle_screen_node") {
        RCLCPP_INFO(this->get_logger(), "RoverViz Start");


        // initializeVisualTools();

    // Allow time for RViz to start
    rclcpp::sleep_for(std::chrono::seconds(2));
    base_link_visual_tool.reset(new rvt::RvizVisualTools("base_link","/base_link_markers", dynamic_cast<rclcpp::Node*>(this)));
    left_bogie_visual_tool.reset(new rvt::RvizVisualTools("left_bogie_link","/base_link_markers", dynamic_cast<rclcpp::Node*>(this)));
    // left_front_visual_tool.reset(new rvt::RvizVisualTools("left_front_wheel_link", "/left_front_wheel_link_markers", dynamic_cast<rclcpp::Node*>(this)));
    // left_middle_visual_tool.reset(new rvt::RvizVisualTools("left_mid_wheel_link", "/left_middle_wheel_link_markers", dynamic_cast<rclcpp::Node*>(this)));
    // right_bogie_visual_tool.reset(new rvt::RvizVisualTools("right_bogie_link", "/right_bogie_link_markers", dynamic_cast<rclcpp::Node*>(this)));
    // right_front_visual_tool.reset(new rvt::RvizVisualTools("right_front_wheel_link", "/right_front_wheel_link_markers", dynamic_cast<rclcpp::Node*>(this)));
    // right_middle_visual_tool.reset(new rvt::RvizVisualTools("right_mid_wheel_link", "/right_middle_wheel_link_markers", dynamic_cast<rclcpp::Node*>(this)));
    // back_bogie_visual_tool.reset(new rvt::RvizVisualTools("back_bogie_link", "/back_bogie_link_markers", dynamic_cast<rclcpp::Node*>(this)));
    // back_right_visual_tool.reset(new rvt::RvizVisualTools("back_right_wheel_link", "/back_right_wheel_link_markers", dynamic_cast<rclcpp::Node*>(this)));
    // back_left_visual_tool.reset(new rvt::RvizVisualTools("back_left_wheel_link", "/back_left_wheel_link_markers", dynamic_cast<rclcpp::Node*>(this)));
  
    base_link_visual_tool->loadMarkerPub();
    left_bogie_visual_tool->loadMarkerPub();
    // left_front_visual_tool->loadMarkerPub();
    // left_middle_visual_tool->loadMarkerPub();
    // right_bogie_visual_tool->loadMarkerPub();
    // right_front_visual_tool->loadMarkerPub();
    // right_middle_visual_tool->loadMarkerPub();
    // back_bogie_visual_tool->loadMarkerPub();
    // back_right_visual_tool->loadMarkerPub();
    // back_left_visual_tool->loadMarkerPub();

    //     bool has_sub = base_link_visual_tool->waitForMarkerPub(10.0);
    //     // has_sub = left_bogie_visual_tool->waitForMarkerPub(10.0);
    //     // has_sub = left_front_visual_tool->waitForMarkerPub(10.0);
    //     // has_sub = left_middle_visual_tool->waitForMarkerPub(10.0);
    //     // has_sub = right_bogie_visual_tool->waitForMarkerPub(10.0);
    //     // has_sub = right_front_visual_tool->waitForMarkerPub(10.0);
    //     // has_sub = right_middle_visual_tool->waitForMarkerPub(10.0);
    //     // has_sub = back_bogie_visual_tool->waitForMarkerPub(10.0);
    //     // has_sub = back_right_visual_tool->waitForMarkerPub(10.0);
    //     // has_sub = back_left_visual_tool->waitForMarkerPub(10.0);
    // if (!has_sub)
    //   RCLCPP_INFO(get_logger(), "/rviz_visual_tools does not have a subscriber after 10s. "
    //                             "Visualizations may be lost");
    // // cam_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    // // // Clear messages
    base_link_visual_tool->deleteAllMarkers();
    left_bogie_visual_tool->deleteAllMarkers();
    // left_front_visual_tool->deleteAllMarkers();
    // left_middle_visual_tool->deleteAllMarkers();
    // right_bogie_visual_tool->deleteAllMarkers();
    // right_front_visual_tool->deleteAllMarkers();
    // right_middle_visual_tool->deleteAllMarkers();
    // back_bogie_visual_tool->deleteAllMarkers();
    // back_right_visual_tool->deleteAllMarkers();
    // back_left_visual_tool->deleteAllMarkers();

    base_link_visual_tool->enableBatchPublishing();
    left_bogie_visual_tool->enableBatchPublishing();
    // left_front_visual_tool->enableBatchPublishing();
    // left_middle_visual_tool->enableBatchPublishing();
    // right_bogie_visual_tool->enableBatchPublishing();
    // right_front_visual_tool->enableBatchPublishing();
    // right_middle_visual_tool->enableBatchPublishing();
    // back_bogie_visual_tool->enableBatchPublishing();
    // back_right_visual_tool->enableBatchPublishing();
    // back_left_visual_tool->enableBatchPublishing();


        // back_bogie_visual_tool->trigger();
        // back_right_visual_tool->trigger();
        // back_left_visual_tool->trigger();
    base_link_visual_tool->setLifetime(UPDATE_INTERVAL_SECONDS);
    left_bogie_visual_tool->setLifetime(UPDATE_INTERVAL_SECONDS);
    left_front_visual_tool->setLifetime(UPDATE_INTERVAL_SECONDS);
    left_middle_visual_tool->setLifetime(UPDATE_INTERVAL_SECONDS);
    right_bogie_visual_tool->setLifetime(UPDATE_INTERVAL_SECONDS);
    right_front_visual_tool->setLifetime(UPDATE_INTERVAL_SECONDS);
    right_middle_visual_tool->setLifetime(UPDATE_INTERVAL_SECONDS);
    back_bogie_visual_tool->setLifetime(UPDATE_INTERVAL_SECONDS);
    back_right_visual_tool->setLifetime(UPDATE_INTERVAL_SECONDS);
    back_left_visual_tool->setLifetime(UPDATE_INTERVAL_SECONDS);



        // camera_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("tf", 10);
        camera_pose_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&RoverViz::update_camera_pose, this));
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

    std::string rover_status = "OFFLINE";

    rvt::RvizVisualToolsPtr base_link_visual_tool;
    rvt::RvizVisualToolsPtr left_bogie_visual_tool;
    rvt::RvizVisualToolsPtr left_front_visual_tool;
    rvt::RvizVisualToolsPtr left_middle_visual_tool;
    rvt::RvizVisualToolsPtr right_bogie_visual_tool;
    rvt::RvizVisualToolsPtr right_front_visual_tool;
    rvt::RvizVisualToolsPtr right_middle_visual_tool;
    rvt::RvizVisualToolsPtr back_bogie_visual_tool;
    rvt::RvizVisualToolsPtr back_right_visual_tool;
    rvt::RvizVisualToolsPtr back_left_visual_tool;

    rclcpp::TimerBase::SharedPtr camera_pose_timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr camera_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> cam_tf_broadcaster_;
};
