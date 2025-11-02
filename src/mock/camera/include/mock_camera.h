#include <rover_utils/include/roverCommon.h>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

#define MAX_FPS 30
#define DEFAULT_FPS 25
#define DEFAULT_HEIGHT 1080
inline constexpr float DEFAULT_RATIO = 16.0/9.0;

class MockCameraNode : public rclcpp::Node 
{
public:
    MockCameraNode();

private:

    // TIMER
    rclcpp::TimerBase::SharedPtr timer_; // For FPS timer

    int fps = 0; // Defined in params
    
    struct CameraConfig {
        std::string id;
        int fps;
        int height;
        int width;
        image_transport::Publisher publisher;
        rclcpp::Time last_pub_time;
        float aspect_ratio;
    };
    std::vector<CameraConfig> cameras_;
    
    void publish_images();

};