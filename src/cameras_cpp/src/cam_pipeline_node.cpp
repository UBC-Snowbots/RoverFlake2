// This is to be a pipeline junction node. Takes in 2 or more camera topics, then reroutes to a topic based on what the user wants
#include <cam_pipeline_node.h>

//* Callbacks

void CamPipe::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg, int index){
    // if(index == selected_index){
    //     sensor_msgs::msg::Image out_msg;
    //     out_msg.height = msg->height;
    //     output_pub->publish(out_msg);
    // }
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CamPipe>();

    // Example: send a command with a steering angle of 0.5 rad and speed of 1.0 m/s

    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}