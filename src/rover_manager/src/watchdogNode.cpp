#include <watchdogNode.h>


void WatchdogNode::cycleNodeHealth(){
auto nodes_currently_running = this->get_node_graph_interface()->get_node_names_and_namespaces();

int num_current_nodes = nodes_currently_running.size();

RCLCPP_INFO(this->get_logger(), "Nodes Currently Running: %i", num_current_nodes);

// nodes_currently_running.

}


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<WatchdogNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}



