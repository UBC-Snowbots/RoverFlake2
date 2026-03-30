#include "rover_vision_cpp/object.h"
#include <fstream>

// YOLOv8 ONNX output: [1, 4+num_classes, 8400] -- row-major, needs transpose
static constexpr int kAnchors = 8400;

YOLODetectorNode::YOLODetectorNode(const rclcpp::NodeOptions &options)
    : Node("yolo_detector_node", options)
{
    declare_parameter("model_path", "bestHammer.onnx");
    declare_parameter("classes_path", "classes.txt");
    declare_parameter("conf_threshold", 0.4);
    declare_parameter("nms_threshold", 0.45);
    declare_parameter("input_size", 640);

    const auto model_path = get_parameter("model_path").as_string();
    const auto classes_path = get_parameter("classes_path").as_string();
    conf_threshold_ = static_cast<float>(get_parameter("conf_threshold").as_double());
    nms_threshold_ = static_cast<float>(get_parameter("nms_threshold").as_double());
    input_size_ = get_parameter("input_size").as_int();

    // Load class names
    std::ifstream f(classes_path);
    // if (!f.is_open())
    // {
    //     RCLCPP_WARN(get_logger(), "Could not open classes file: %s -- class names will be indices",
    //                 classes_path.c_str());
    // }
    // else
    // {
    //     std::string line;
    //     while (std::getline(f, line))
    //     {
    //         if (!line.empty())
    //             class_names_.push_back(line);
    //     }
    // }

    // Load ONNX model
    net_ = cv::dnn::readNetFromONNX(model_path);
    if (net_.empty())
    {
        throw std::runtime_error("Failed to load ONNX model: " + model_path);
    }
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    RCLCPP_INFO(get_logger(), "Loaded model: %s", model_path.c_str());

    pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/yolo/object_detection", 10);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", 10, // Fine for now, lets assume this is always used
        std::bind(&YOLODetectorNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "YOLODetectorNode ready");
}

// -----------------------------------------------------------------------------

void YOLODetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Decode ROS image to cv::Mat
    cv::Mat frame(msg->height, msg->width, CV_8UC3,
                  const_cast<uint8_t *>(msg->data.data()));

    if (msg->encoding == "rgb8")
    {
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    }

    auto detections = runInference(frame);
    RCLCPP_INFO(get_logger(), "Inference successfully run");

    // Draw
    cv::Mat annotated = frame.clone();
    for (const auto &det : detections)
    {
        const auto colour = classColour(det.class_id);
        cv::rectangle(annotated, det.box, colour, 2);

        std::string label = (det.class_id < static_cast<int>(class_names_.size()))
                                ? class_names_[det.class_id]
                                : std::to_string(det.class_id);
        label += " " + std::to_string(static_cast<int>(det.confidence * 100)) + "%";

        cv::putText(annotated, label,
                    {det.box.x, det.box.y - 5},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, colour, 2);
    }

    // Publish
    sensor_msgs::msg::Image out;
    out.header = msg->header;
    out.height = annotated.rows;
    out.width = annotated.cols;
    out.encoding = "bgr8";
    out.step = annotated.cols * 3;
    out.data.assign(annotated.data, annotated.data + annotated.total() * annotated.elemSize());
    pub_->publish(out);
}

// -----------------------------------------------------------------------------

std::vector<Detection> YOLODetectorNode::runInference(const cv::Mat &frame)
{
    cv::Mat blob = preprocess(frame);
    net_.setInput(blob);
    RCLCPP_INFO(get_logger(), "everything except pre-process.. ");
    std::vector<cv::Mat> outputs;
    RCLCPP_INFO(get_logger(), "1");
    net_.forward(outputs, net_.getUnconnectedOutLayersNames());
    RCLCPP_INFO(get_logger(), "everything except post process.. ");
    RCLCPP_INFO(get_logger(), "Output dims: %d", outputs[0].dims);
    // RCLCPP_INFO(get_logger(), outputs);

    for (int i = 0; i < outputs[0].dims; ++i)
    {
        RCLCPP_INFO(get_logger(), "  dim[%d] = %d", i, outputs[0].size[i]);
    }
    return postprocess(outputs[0], frame.cols, frame.rows);
}

cv::Mat YOLODetectorNode::preprocess(const cv::Mat &frame)
{
    // letterbox resize to maintain aspect ratio
    int sz = input_size_;
    float scale = std::min(static_cast<float>(sz) / frame.cols,
                           static_cast<float>(sz) / frame.rows);
    int new_w = static_cast<int>(frame.cols * scale);
    int new_h = static_cast<int>(frame.rows * scale);

    cv::Mat resized;
    cv::resize(frame, resized, {new_w, new_h});

    cv::Mat padded(sz, sz, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(padded(cv::Rect(0, 0, new_w, new_h)));

    return cv::dnn::blobFromImage(padded, 1.0 / 255.0, {sz, sz}, {0, 0, 0}, true);
}

std::vector<Detection> YOLODetectorNode::postprocess(
    const cv::Mat &output, int orig_w, int orig_h)
{
    RCLCPP_INFO(get_logger(), "Post-processing.. ");

    // cv::Mat out;
    cv::Mat out = output.reshape(1, output.size[1]).t(); // [8400, cols]
    try
    {
        out = output.reshape(1, output.size[1]).t();
    }
    catch (const cv::Exception &e)
    {
        RCLCPP_ERROR(get_logger(), "reshape failed: %s", e.what());
        return {};
    }

    int num_classes = out.cols - 4;
    float scale_x = static_cast<float>(orig_w) / input_size_;
    float scale_y = static_cast<float>(orig_h) / input_size_;
    RCLCPP_INFO(get_logger(), "Output dims: %d", output.dims);
    for (int i = 0; i < output.dims; ++i)
    {
        RCLCPP_INFO(get_logger(), "  dim[%d] = %d", i, output.size[i]);
    }
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;

    for (int i = 0; i < out.rows; ++i)
    {
        const float *row = out.ptr<float>(i);

        // Find max class score
        cv::Mat scores(1, num_classes, CV_32F, const_cast<float *>(row + 4));
        double max_score;
        cv::Point max_loc;
        cv::minMaxLoc(scores, nullptr, &max_score, nullptr, &max_loc);

        if (static_cast<float>(max_score) < conf_threshold_)
            continue;

        // cx, cy, w, h → x1, y1, w, h
        float cx = row[0] * scale_x;
        float cy = row[1] * scale_y;
        float w = row[2] * scale_x;
        float h = row[3] * scale_y;

        boxes.push_back({static_cast<int>(cx - w / 2),
                         static_cast<int>(cy - h / 2),
                         static_cast<int>(w),
                         static_cast<int>(h)});
        confidences.push_back(static_cast<float>(max_score));
        class_ids.push_back(max_loc.x);
    }

    // NMS
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, indices);

    std::vector<Detection> result;
    result.reserve(indices.size());
    for (int idx : indices)
    {
        result.push_back({boxes[idx], confidences[idx], class_ids[idx]});
    }
    return result;
}

cv::Scalar YOLODetectorNode::classColour(int class_id)
{
    static const std::vector<cv::Scalar> palette = {
        {255, 56, 56},
        {255, 157, 151},
        {255, 112, 31},
        {255, 178, 29},
        {207, 210, 49},
        {72, 249, 10},
        {146, 204, 23},
        {61, 219, 134},
        {26, 147, 52},
        {0, 212, 187},
        {44, 153, 168},
        {0, 194, 255},
        {52, 69, 147},
        {100, 115, 255},
        {0, 24, 236},
        {132, 56, 255},
        {82, 0, 133},
        {203, 56, 255},
        {255, 149, 200},
        {255, 55, 199},
    };
    return palette[class_id % palette.size()];
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YOLODetectorNode>());
    rclcpp::shutdown();
    return 0;
}