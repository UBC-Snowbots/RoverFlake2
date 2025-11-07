#include <mock_camera.h>


MockCameraNode::MockCameraNode() : Node("mock_camera_node", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();


    // Get all declared parameters (flattened)
    auto all_params = this->list_parameters({}, 10);  // depth=10 → include nested ones

    // Find unique camera IDs: "cam1", "cam2", ...
    std::set<std::string> camera_ids;
    for (auto &name : all_params.names) {
        if (name.rfind("cameras.", 0) == 0) {  // starts with "cameras."
            auto sub = name.substr(std::string("cameras.").size());
            auto dot_pos = sub.find('.');
            if (dot_pos != std::string::npos)
                camera_ids.insert(sub.substr(0, dot_pos));
        }
    }
    
    if (camera_ids.empty()) {
        RCLCPP_WARN(this->get_logger(), "No cameras found in parameters, defaulting to 'cam1'");
        camera_ids.insert("cam1");
    }

    for (const auto &id : camera_ids) {
        CameraConfig cfg;
        
        // Don't love this.
        std::string param_name("cameras." + id + ".fps");
        if (this->has_parameter(param_name))
        {
            cfg.fps = this->get_parameter(param_name).get_value<int>();
        } else {
            cfg.fps = DEFAULT_FPS;
        }
        param_name = std::string("cameras." + id + ".height");
        if (this->has_parameter(param_name))
        {
            cfg.height = this->get_parameter(param_name).get_value<int>();
        } else {
            cfg.height = DEFAULT_HEIGHT;
        }
        param_name = std::string("cameras." + id + ".ratio");
        if (this->has_parameter(param_name))
        {
            cfg.aspect_ratio = this->get_parameter(param_name).get_value<float>();
        } else {
            cfg.aspect_ratio = DEFAULT_RATIO;
        }
        cfg.id = id;
        cfg.width = static_cast<int>((cfg.height * cfg.aspect_ratio));

        cfg.publisher = image_transport::create_publisher(this, cfg.id);
        cameras_.push_back(cfg);

        RCLCPP_INFO(this->get_logger(),
                    "Mock Camera '%s' loaded (fps=%d, res=%d)",
                    cfg.id.c_str(), cfg.fps, cfg.height);
    }
    
    // start a timer to publish mock images
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / MAX_FPS),
                                        std::bind(&MockCameraNode::publish_images, this));
}

int main(int argc, char** argv)
{
 rclcpp::init(argc, argv);
 auto node = std::make_shared<MockCameraNode>();
 rclcpp::spin(node);
 
 rclcpp::shutdown();
 return 0;
}

void MockCameraNode::publish_images() {
    for (auto &cam : cameras_) {
        int ms_since_last_pub = (this->get_clock()->now().nanoseconds() - cam.last_pub_time.nanoseconds()) / NANOSECONDS_PER_MILLISECOND;
        int current_ms = (this->get_clock()->now().nanoseconds() % 1'000'000'000) / NANOSECONDS_PER_MILLISECOND; // Current ms since last second (used to draw rectangle)
        if( ms_since_last_pub > 1000 /cam.fps)
        {
            char buffer[64];
            std::snprintf(buffer, sizeof(buffer), "FPS: %d", cam.fps);
            std::string fps_text(buffer);
            cv::Mat img(cam.height, cam.width, CV_8UC3, cv::Scalar(100, 100, 100));
            cv::putText(img, cam.id, cv::Point(30, 50),
                cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255,255,255), 2);
            cv::putText(img, fps_text, cv::Point(30, 100),
                cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255,255,255), 2);
            cv::Point rect_pt1((cam.width/2) - (current_ms * (static_cast<float>(cam.height)/(MILLISECONDS_PER_SECOND))), (cam.height/2) - 30);
            cv::Point rect_pt2((cam.width/2) + (current_ms * (static_cast<float>(cam.height)/(MILLISECONDS_PER_SECOND))), (cam.height/2) + 30);

            cv::rectangle(img, rect_pt1, rect_pt2,
                            cv::Scalar(0,0,0), 3);
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
            msg->header.stamp = this->get_clock()->now();
            msg->header.frame_id = cam.id;
            cam.publisher.publish(*msg);
            cam.last_pub_time = msg->header.stamp;
        }
    }
}

