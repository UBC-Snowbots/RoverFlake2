#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/arm_command.hpp>

namespace rvt = rviz_visual_tools;



class RoverViz : public rclcpp::Node {
public:
    RoverViz() : Node("middle_screen_node") {
        RCLCPP_INFO(this->get_logger(), "RoverViz Start");


        // initializeVisualTools();

    // Allow time for RViz to start
    rclcpp::sleep_for(std::chrono::seconds(1));
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_frame","/rviz_visual_markers", this));
    //   visual_tools_->loadMarkerPub();
    //       bool has_sub = visual_tools_->waitForMarkerSub(10.0);
    // if (!has_sub)
    //   RCLCPP_INFO(get_logger(), "/rviz_visual_tools does not have a subscriber after 10s. "
    //                             "Visualizations may be lost");

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

    rvt::RvizVisualToolsPtr visual_tools_;
};
