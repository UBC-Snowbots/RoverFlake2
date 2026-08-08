// gnss_mission_module.h — "GNSS Mission"
//
// Front panel for rover_mapping's mission_manager: start/stop a recorded
// mission, tag waypoints at the current fix, open/close route segments —
// all over its services — plus an embedded offline tile map (GnssMapWidget)
// that live-plots the fix path and tags, and report export via the `rover`
// CLI. Section: Tasks. defaultVisible: false.

#pragma once

#include <rover_hmi_core/gui_module.h>
#include <rover_hmi_core/tasks/gnss_map_widget.h>

#include <QLabel>
#include <QLineEdit>
#include <QProcess>

#include <rover_mapping_interfaces/srv/export_map.hpp>
#include <rover_mapping_interfaces/srv/segment.hpp>
#include <rover_mapping_interfaces/srv/start_mission.hpp>
#include <rover_mapping_interfaces/srv/tag_point.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

class GnssMissionModule : public rover_hmi_core::GuiModule {
    using StartMission = rover_mapping_interfaces::srv::StartMission;
    using TagPoint     = rover_mapping_interfaces::srv::TagPoint;
    using Segment      = rover_mapping_interfaces::srv::Segment;
    using ExportMap    = rover_mapping_interfaces::srv::ExportMap;
    using Trigger      = std_srvs::srv::Trigger;

public:
    std::string name()        const override { return "GNSS Mission"; }
    std::string sectionName() const override { return "Tasks"; }
    std::string layoutHint()  const override { return "bottom"; }
    bool        defaultVisible() const override { return false; }

    void     setNode(rclcpp::Node::SharedPtr node) override;
    QWidget* createWidget(QWidget* parent) override;

private:
    void startMission();
    void stopMission();
    void tag(const QString& category);
    void segment(bool open);
    void exportReport();
    void openReport();
    void fetchTiles();
    void report(bool ok, const QString& msg);
    void refreshFix();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr       active_sub_;
    rclcpp::Client<StartMission>::SharedPtr start_cli_;
    rclcpp::Client<Trigger>::SharedPtr      stop_cli_;
    rclcpp::Client<TagPoint>::SharedPtr     tag_cli_;
    rclcpp::Client<Segment>::SharedPtr      seg_start_cli_;
    rclcpp::Client<Segment>::SharedPtr      seg_end_cli_;
    rclcpp::Client<ExportMap>::SharedPtr    export_cli_;
    QString last_report_;

    sensor_msgs::msg::NavSatFix::SharedPtr fix_;
    rclcpp::Time fix_stamp_;

    QLabel*        fix_label_     = nullptr;
    QLabel*        mission_label_ = nullptr;
    QLabel*        status_        = nullptr;
    QLineEdit*     mission_name_  = nullptr;
    QLineEdit*     tag_label_     = nullptr;
    QLineEdit*     seg_name_      = nullptr;
    GnssMapWidget* map_           = nullptr;
    QProcess*      fetch_proc_    = nullptr;
};
