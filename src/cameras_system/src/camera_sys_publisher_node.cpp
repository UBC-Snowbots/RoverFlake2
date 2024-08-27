#include "cameras_system/camera_sys_publisher_node.hpp"

CamerasSysNode::CamerasSysNode()
: Node("camera_sys_publisher_node"), last_msg_(nullptr) {
    // Initialize Publisher
    pub_fcam_ = image_transport::create_publisher(this, "fcam/compressed_feed");
    // pub_bcam_ = image_transport::create_publisher(this, "bcam/compressed_feed");

    // Create subscription
    msg_sub_ = this->create_subscription<std_msgs::msg::String>(
        "byte_msg", 10, std::bind(&CamerasSysNode::topic_callback, this, std::placeholders::_1));

    // Basic timer implementation
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&CamerasSysNode::timer_callback, this));

    // Open cameras with GStreamer pipelines for raw capture KEEP FOR NOW
    capf_.open("v4l2src device=/dev/video0 ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! videoconvert ! appsink",
                            cv::CAP_GSTREAMER);
    // capb_.open("v4l2src device=/dev/video4 ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! videoconvert ! appsink",
    //                         cv::CAP_GSTREAMER);

    if (!capf_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Front Camera Failed to open");
    }

    // if (!capb_.isOpened()) {
    //     RCLCPP_ERROR(this->get_logger(), "Back Camera Failed to open");
    // }

    //  // Initialize VideoWriter for H.264 encoding
    // writer_fcam_.open("appsrc ! videoconvert ! x264enc tune=zerolatency ! video/x-h264,profile=baseline ! appsink", cv::CAP_GSTREAMER, 0, 30, cv::Size(640, 480), true);
    // writer_bcam_.open("appsrc ! videoconvert ! x264enc tune=zerolatency ! video/x-h264,profile=baseline ! appsink", cv::CAP_GSTREAMER, 0, 30, cv::Size(640, 480), true);

    // if (!writer_fcam_.isOpened()) {
    //     RCLCPP_ERROR(this->get_logger(), "Front Camera VideoWriter Failed to open");
    // }

    // if (!writer_bcam_.isOpened()) {
    //     RCLCPP_ERROR(this->get_logger(), "Back Camera VideoWriter Failed to open");
    // }
}

CamerasSysNode::~CamerasSysNode() {
    capf_.release();
    // capb_.release();
    // writer_fcam_.release();
    // writer_bcam_.release();
}

void CamerasSysNode::timer_callback() {
    // Check if a message has been received
    if (!last_msg_) {
        // If no message has been received yet, do nothing
        return;
    }

    // Init frames
    cv::Mat fframe; //, bframe;

    // Encode YUY2 -> H264 format
    std::vector<uchar> encoded_fframe;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 95};
    cv::imencode(".jpg", fframe, encoded_fframe, params);

    // ROS message
    auto frontmsg = sensor_msgs::msg::CompressedImage();
    frontmsg.header.stamp = this->get_clock()->now();
    frontmsg.format = "jpeg";
    frontmsg.data = std::move(encoded_fframe);

    // Init byte msg
    std::bitset<3> byte_msg(last_msg_->data.c_str()); // change later
    int byte_str = static_cast<int>(byte_msg.to_ulong());

    // Check bitvals
    if ((byte_str & (1 << 2)) == (1 << 2)){
        // initial bit is on, so proceed with sending camera info
        // byte info will look like 10000000 from l-r <on, front, back, left, right, arm1, arm2, ptz>
        // Capture frames from the cameras if 1
        if ((byte_str & (1 << 1)) == (1 << 1)){
            // Front
            if (capf_.isOpened() && capf_.read(fframe)) {
                // writer_fcam_.write(fframe);
                // auto frontmsg = cv_mat_to_compressed_image_msg(fframe, "fcam/compressed_feed");
                pub_fcam_.publish(frontmsg);
            }
        }
        if ((byte_str & (1 << 0)) == (1 << 0)){
            // Back
            // if (capb_.isOpened() && capb_.read(bframe)) {
            //     // writer_bcam_.write(bframe);
            //     // auto backmsg = cv_mat_to_compressed_image_msg(bframe, "bcam/compressed_feed");
            //     // pub_bcam_.publish(backmsg);
                
            // }
        }
    }
    else {
        RCLCPP_WARN(this->get_logger(), "Msg validation key invalid");
    }
}

void CamerasSysNode::topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    last_msg_ = msg;
}

// sensor_msgs::msg::CompressedImage::SharedPtr CamerasSysNode::cv_mat_to_compressed_image_msg(const cv::Mat & frame, const std::string & frame_id) {
//     // Init msg data
//     std_msgs::msg::Header header;
//     header.stamp = this->now();
//     header.frame_id = frame_id;

//     // Encode image to memory
//     std::vector<uchar> buf;
//     cv::imencode(".jpg", frame, buf);

//     auto img_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
//     img_msg->header = header;
//     img_msg->format = "jpeg";
//     img_msg->data.assign(buf.begin(), buf.end());

//     return img_msg;
// }

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CamerasSysNode>());
    rclcpp::shutdown();
    return 0;
}
