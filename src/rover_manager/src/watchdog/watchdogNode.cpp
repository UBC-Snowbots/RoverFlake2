#include <watchdogNode.h>
//* Created a while ago and kinda forgotten. 

void WatchdogNode::cycleNodeHealth(){
auto nodes_currently_running = this->get_node_graph_interface()->get_node_names_and_namespaces();

int num_current_nodes = nodes_currently_running.size();
std::vector<std::string> expected_nodes = this->get_parameter("expected_nodes").as_string_array();

RCLCPP_INFO(this->get_logger(), "Nodes Currently Running: %i", num_current_nodes);

for(auto node : nodes_currently_running){
    RCLCPP_INFO(this->get_logger(), "%s", node.first.c_str());
    for(auto expected_node : this->get_parameter("expected_nodes").as_string_array()){
    if(expected_node.size() > 0){
            if(expected_node == node.first){
        break;
    }else{
        RCLCPP_ERROR(this->get_logger(), "NODE NOT RUNNING WHEN IT SHOULD BE: %s", node.first.c_str());
    }
    }else{
        RCLCPP_ERROR(this->get_logger(), "expected_nodes Parameter Empty");
    }

}


}

}






int main(int argc, char *argv[]){
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<WatchdogNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}



