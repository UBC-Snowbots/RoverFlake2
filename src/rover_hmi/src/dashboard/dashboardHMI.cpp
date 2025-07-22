#include <dashboardHMI.h>

#define DEBUG_MSGS

int main(int argc, char* argv[]){
    //Gtk is a picky eater and crashes when you feed it ros args, which are automatically fed with launch file, and i think regular running.
    // So, GTK gets null args. 
    int nullc = 0;
    char **nullv = nullptr;

    auto app = Gtk::Application::create(nullc, nullv, "dashboard_hmi"); //Make sure the 3rd arg here is unique. If 2 HMIs start with the same id one will die
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

/**
 * @brief callback for 'heartbeats' - Will update HMI based on what hosts and systems are running/offline
 */
void DashboardHMINode::heartFeedbackCallback(const rover_msgs::msg::HeartRequest::SharedPtr msg){
    //Watchdog / Heartbeat callback. See what systems are running
    #ifdef DEBUG_MSGS
        RCLCPP_INFO(this->get_logger(), "Heartbeat Detected: %s, on host %s", msg->subsystem_name.c_str(), msg->subsystem_host.c_str());
    #endif

    const std::string host = msg->subsystem_host; // Computer running a heart
    const std::string subsys_name = msg->subsystem_name;
    auto now = this->get_clock()->now();
    auto time_since_last_heartbeat = now - msg->header.stamp;
    
    Glib::RefPtr<Gtk::StyleContext> subsys_context;
    Gtk::Label* status_label;
    if(host == MONITORED_COMPUTER_CONTROL_BASE_STRING){
        subsys_context = monitored_systems_control_base[subsys_name].status_label->get_style_context();
        status_label = monitored_systems_control_base[subsys_name].status_label;
    }
    if(host == MONITORED_COMPUTER_ONBOARD_JETSON_STRING){
        subsys_context = monitored_systems_onboard_jetson[subsys_name].status_label->get_style_context();
        status_label = monitored_systems_onboard_jetson[subsys_name].status_label;
      
    }
    if(host == MONITORED_COMPUTER_ONBOARD_NUC_STRING){
        subsys_context = monitored_systems_onboard_nuc[subsys_name].status_label->get_style_context();
        status_label = monitored_systems_onboard_nuc[subsys_name].status_label;

    }
    subsys_context->remove_class("subsys_OFFLINE");
    subsys_context->remove_class("subsys_ONLINE");
    
    if(msg->running){
        subsys_context->add_class("subsys_ONLINE");
        status_label->set_label("ONLINE");

    }else{
        
        subsys_context->add_class("subsys_OFFLINE");
        status_label->set_label("OFFLINE");

    }



}

void DashboardHMINode::subsystemRequest(std::string subsystem_name, int request, int computer){
    RCLCPP_INFO(this->get_logger(), "Button Clicked");
    //* Depending on the button clicked, run different child proccesses, or kill different child processes
    // runChildNode("rviz2", "rviz2");
    // runChildNode("joy", "joy_node");
    rover_msgs::msg::HeartRequest msg;
    msg.subsystem_name = subsystem_name;
    if(request == RUN){
        // runSubSystem(subsystem_name);
        // runChildNode("rover_launchers", "ps4.launch.py", "control_base", true);
        msg.running = true;
    }
    if(request == KILL){
        // killSubSystem(subsystem_name);
        msg.running = false;
    }
    switch (computer)
    {
        case COMPUTER_CONTROL_BASE:
        // control_base_heart_request_pub->publish(msg);
        break;
        case COMPUTER_ONBOARD_NUC:
        // onboard_nuc_heart_request_pub->publish(msg);
        break;
        case COMPUTER_GLOBAL: //? one topic for all computers. Just rely on subsystem names to ensure what runs where. Each computer knows what subsystems it should run in the params.
        global_heart_request_pub->publish(msg);
        break;
        default:
        RCLCPP_ERROR(this->get_logger(), "Computer unkown. Subprocess start failure");
        break;
    }
    
    
}

    
bool DashboardHMINode::handleSubsystemStatusGridDraw(const Cairo::RefPtr<Cairo::Context>& context, int computer){
    switch(computer){
        case computer::control_base:
            
            break;
        case computer::onboard_nuc:
            break;
        default:
            break;
    }
}


//* Archive Code (graveyard)


// void DashboardHMINode::runChildNode(std::string pkg, std::string node_or_launch_file, std::string subsystem_name, int type, bool kill_orphan){
// //? This will run nodes in parallell threads and attach them as children to this process.
//         //* Will run launch files if launch is true. When running a launch file:
//             //* This node is the 'parent'
//             //* the launch file is the 'child'
//             //* the nodes within the launch file are like "grandchildren"
//             //* Killing the launch file process will automatically kill all nodes launched by that launch file
//             //* So, use launch files to group together nodes that you don't need to start/kill independently
// //? After running a new child node, its PID is saved in the hash map, and can be killed, or paused by this node
// //? if dashboard dies, will default to deystroy it's children, 
//             //*  unless kill_orphan is set to false
//         if(monitored_systems[subsystem_name].online == true){
//             RCLCPP_ERROR(this->get_logger(), "Subystem process %s is already running. Will not respawn", subsystem_name.c_str());
//             return;
//         }

//         pid_t pid = fork();//* once this is called, both the child and parent run the code below, with different outcomes
//         if (pid == 0) { //if fork was successful, and I'm the child
//             // Child process: replace this process with "ros2 run my_package my_node"
//             if (setsid() < 0) {
//                 perror("setsid failed");
//                 exit(EXIT_FAILURE);
//             }
//             pid_t child_sid = getsid(0);
//             pid_t child_gpid = getpgrp();
//             RCLCPP_INFO(this->get_logger(), "Launched process with PID: %d, GPID: %d, SID: %d\n", pid, child_gpid, child_sid);

//             if(type == LAUNCHFILE){

//                 execlp("ros2", "ros2", "launch", pkg.c_str(), node_or_launch_file.c_str(), (char *)NULL);
//             }else{
//                 execlp("ros2", "ros2", "run", pkg.c_str(), node_or_launch_file.c_str(), (char *)NULL);
//             }
    
//             // If execlp() returns, there was an error. So this code should be blocked if all goes well.
//             perror("execlp failed");
//             exit(EXIT_FAILURE);
    
//         } else if (pid < 0) {
//             // fork() error in parent
//             perror("fork failed");
//         } else {
//             // Parent process: fork succeeded
//             // 'pid' is the child's PID. We could store it if we want to terminate later.
//             pid_t child_sid = pid;
//             pid_t child_gpid = pid;

//             RCLCPP_INFO(this->get_logger(), "Launched process with PID: %d, GPID: %d, SID: %d\n", pid, child_gpid, child_sid);
//             running_child_pids[node_or_launch_file] = pid;
//             monitored_systems[subsystem_name].pid = pid;
//             monitored_systems[subsystem_name].sid = pid;
//             monitored_systems[subsystem_name].gpid = pid;
//             monitored_systems[subsystem_name].online = true;
//         }


// }

// void DashboardHMINode::killSubSystem(std::string subsystem_name){
//     if(monitored_systems[subsystem_name].online){
//         killProcessGroup(monitored_systems[subsystem_name].gpid);
//         monitored_systems[subsystem_name].online = false;
//         Glib::RefPtr<Gtk::StyleContext> context = monitored_systems[subsystem_name].status_label->get_style_context();
//         context->remove_class("subsys_ONLINE");
//         context->add_class("subsys_OFFLINE");
//         monitored_systems[subsystem_name].status_label->set_label("OFFLINE");

//     }else{
//         RCLCPP_ERROR(this->get_logger(), "Subsystem %s is already offline", subsystem_name.c_str());
//     }
// }

// void DashboardHMINode::runSubSystem(std::string subsystem_name){
//     // for(const auto &process : monitored_systems[subsystem_name].processes){
//         monitored_systems[subsystem_name].status_label->set_label("STARTING");
//         const auto &process = monitored_systems[subsystem_name].process;
//         runChildNode(process.pkg, process.exec, subsystem_name, process.type);
//         Glib::RefPtr<Gtk::StyleContext> context = monitored_systems[subsystem_name].status_label->get_style_context();
//         context->add_class("subsys_ONLINE");
//         context->remove_class("subsys_OFFLINE");
//         monitored_systems[subsystem_name].status_label->set_label("online");


//     // }
// }

// void DashboardHMINode::killProcessGroup(pid_t pgid) {
//     RCLCPP_INFO(this->get_logger(), "Killing process group: %d", pgid);

//     pid_t my_pgid = getpgrp();
//     if(pgid == my_pgid || pgid == 0){
//         RCLCPP_ERROR(this->get_logger(), "Holdup, halted attempt to kill my own proccess group. %d",my_pgid);
//         return;
//     }
    
//     // Send SIGTERM to the entire process group
//     if (killpg(pgid, SIGINT) < 0) {
//         RCLCPP_ERROR(this->get_logger(), "Error sending SIGKILL to process group %d: %s", 
//                     pgid, strerror(errno));
//     }
// }
// void DashboardHMINode::killChildPID(pid_t target_pid){
//     RCLCPP_INFO(this->get_logger(), "Killing: %d", target_pid);
//     kill(target_pid, SIGINT); //* nicely ask
//     // kill(pid, SIGKILL); //* demand death
// }

// std::vector<pid_t> DashboardHMINode::getPidsByName(const std::string &processName, bool verbose)
// {
//     std::vector<pid_t> pids;

//     // Build our command string, e.g.: "pgrep my_node"
//     // -f matches against the entire command line, 
//     // so you might do: "pgrep -f " + processName, depending on your usage
//     const std::string cmd = "pgrep " + processName;

//     // Open a pipe to read the results of pgrep
//     FILE* pipe = popen(cmd.c_str(), "r");
//     if (!pipe) {
//         std::perror("popen failed");
//         return pids;
//     }

//     char buffer[128];
//     while (fgets(buffer, sizeof(buffer), pipe)) {
//         // Each line should contain one PID
//         pid_t pid = static_cast<pid_t>(std::stoi(buffer));
//         pids.push_back(pid);
//     }

//     pclose(pipe);



//     return pids;
// }
