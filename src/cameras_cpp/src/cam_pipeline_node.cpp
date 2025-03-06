// This is to be a pipeline junction node. Takes in 2 or more camera topics, then reroutes to a topic based on what the user wants
// I'm sure there is another way of doing this, but this is the most controllable way for our purposes
#include <cam_pipeline_node.h>

//* Callbacks - If using multithreaded executor, we might need to worry about concurrency and thread safety stuff

void CamPipe::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg, int index){
    if(index == selected_index){
        output_pub->publish(*msg); //? SEND IT TO BASE!
    }else{
        //? DROP IT SO THE NETWORK ONLY SENDS WHAT IT NEEDS TO
        RCLCPP_ERROR(this->get_logger(), "Camera index %i is out of bounds", index);
    }
}

void CamPipe::selectIndexCallback(const std_msgs::msg::Int16::SharedPtr msg){
    this->selected_index = msg->data;
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CamPipe>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}