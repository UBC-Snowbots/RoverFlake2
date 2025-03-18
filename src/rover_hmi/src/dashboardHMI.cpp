#include <dashboardHMI.h>

#define DEBUG_MSGS

int main(int argc, char* argv[]){
    //Gtk is a picky eater and crashes when you feed it ros args, which are automatically fed with launch file, and i think regular running.
    // So, GTK gets null args. 
    int nullc = 0;
    char **nullv = nullptr;

    auto app = Gtk::Application::create(nullc, nullv, "com.example.GtkApplication");
    //CHAD ROS2 gets the real arguments from the terminal
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DashboardHMINode>();

    sigc::connection spin_connection = Glib::signal_timeout().connect([&node]() -> bool {
        rclcpp::spin_some(node);
        return true;
    }, 20); //MILlISECINDS

    node->app = app;
    node->run();
    spin_connection.disconnect();
    node.reset();
    rclcpp::shutdown();

    return 0;

}


void DashboardHMINode::heartbeatCallback(const rover_msgs::msg::SubSystemHealth::SharedPtr msg){
    //Watchdog / Heartbeat callback. See what systems are running
    #ifdef DEBUG_MSGS
        RCLCPP_INFO(this->get_logger(), "Heartbeat Detected: %s", msg->subsystem);
    #endif
}


void DashboardHMINode::runChildNode(std::string pkg, std::string node_or_launch_file, bool launch, bool kill_orphan){
//? This will run nodes in parallell threads and attach them as children to this process.
        //* Will run launch files if launch is true. When running a launch file:
            //* This node is the 'parent'
            //* the launch file is the 'child'
            //* the nodes within the launch file are like "grandchildren"
            //* Killing the launch file process will automatically kill all nodes launched by that launch file
            //* So, use launch files to group together nodes that you don't need to start/kill independently
//? After running a new child node, its PID is saved in the hash map, and can be killed, or paused by this node
//? if dashboard dies, will default to deystroy it's children, 
            //*  unless kill_orphan is set to false


        pid_t pid = fork();//* once this is called, both the child and parent run the code below, with different outcomes
        if (pid == 0) { //if fork was successful, and I'm the child
            // Child process: replace this process with "ros2 run my_package my_node"
            if(launch){

                execlp("ros2", "ros2", "launch", pkg.c_str(), node_or_launch_file.c_str(), (char *)NULL);
            }else{
                execlp("ros2", "ros2", "run", pkg.c_str(), node_or_launch_file.c_str(), (char *)NULL);
            }
    
            // If execlp() returns, there was an error
            perror("execlp failed");
            exit(EXIT_FAILURE);
    
        } else if (pid < 0) {
            // fork() error in parent
            perror("fork failed");
        } else {
            // Parent process: fork succeeded
            // 'pid' is the child's PID. We could store it if we want to terminate later.
            RCLCPP_INFO(this->get_logger(), "Launched process with PID: %d\n", pid);
            running_child_pids[node_or_launch_file] = pid;
        }


}

void DashboardHMINode::killChildPID(pid_t target_pid){
    RCLCPP_INFO(this->get_logger(), "Killing: %d", target_pid);
    kill(target_pid, SIGTERM); //* nicely ask
    // kill(pid, SIGKILL); //* demand death
}

std::vector<pid_t> DashboardHMINode::getPidsByName(const std::string &processName)
{
    std::vector<pid_t> pids;

    // Build our command string, e.g.: "pgrep my_node"
    // -f matches against the entire command line, 
    // so you might do: "pgrep -f " + processName, depending on your usage
    const std::string cmd = "pgrep " + processName;

    // Open a pipe to read the results of pgrep
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        std::perror("popen failed");
        return pids;
    }

    char buffer[128];
    while (fgets(buffer, sizeof(buffer), pipe)) {
        // Each line should contain one PID
        pid_t pid = static_cast<pid_t>(std::stoi(buffer));
        pids.push_back(pid);
    }

    pclose(pipe);
    return pids;
}