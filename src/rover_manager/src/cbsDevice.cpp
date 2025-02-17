#include "cbsDevice.h"
#include "cbsManagerNode.h"
using namespace ConsoleFormat;
bool CBSDevice::attachPort(std::string port, int baudrate, int id){
if(port.size() == 0){
    RCLCPP_ERROR(manager->get_logger(), "%s Port Open Failure: %s No port name given. Aborting serial connection.", ConsoleFormat::red(), ConsoleFormat::reset());
    delete this; //? A terrible way to stop the program.
    return 0;
}else{
    return 1;
}

}


// void CBSDevice::setLogger(rclcpp::Logger* main_logger){
//     logger_ = main_logger;
//     RCLCPP_INFO(*logger_, "Logger attached. Hello world! im %s", this->cbs_id.c_str());
// }

// void CBSDevice::initalize(rclcpp::Logger* )

int CBSDevice::testPort(std::string port_path, int baudrate){
    
    if(serial.isOpen()){

    }else{
        serial.setPort(port_path);
        serial.setBaudrate(baudrate);
        try{serial.open();
            
            rclcpp::Clock clock(RCL_SYSTEM_TIME);
            rclcpp::Time start_time = clock.now();
            while(serial.available() == 0){
                //wait
                if(clock.now().seconds() - start_time.seconds() > 3){
                    return SERIAL_PORT_TIMEOUT;
                }

            }
            std::string buff;
            bool id_located = false;
            while(!id_located){
                serial.readline(buff);
                size_t start = buff.find("$");
                if(start != std::string::npos){ //If we actually found the position of "$"
                    int end_of_id = buff.find("(");                    
                    std::string id;
                    std::string panel_id = buff.substr(start, end_of_id - start); // Parse id from start of string to end
                    RCLCPP_INFO(manager->get_logger(), "%s %s %s | String ID found at %li: %s'%s'%s", yellow(), this->cbs_id.c_str(), reset(), start, green(), panel_id.c_str(), reset() );
                    id_located = true;
                    // this->id = panel_id;
                }else{
                    RCLCPP_WARN(manager->get_logger(), "%s, Serial buff garbage line found", this->cbs_id.c_str());
                }
                
            }


        } catch(...){
    RCLCPP_INFO(manager->get_logger(), "..%s Port Open Failure: %s No port name given or something is wrong. Aborting serial connection.", ConsoleFormat::red(), ConsoleFormat::reset());
        return SERIAL_PORT_BUSY_OR_DNE;
            }
    }

    
}

void CBSDevice::setID(std::string id){
    this->cbs_id = id;
    // RCLCPP_INFO(manager->get_logger(), "Id set: %s", id.c_str());
}

void CBSDevice::initalize(std::string port_path, int baudrate, std::string id, CBSManagerNode* manager_){
    manager = manager_;
    cbs_id = id;
    RCLCPP_INFO(manager->get_logger(), "Logger attached. Hello world! im %s", this->cbs_id.c_str());

}
void CBSDevice::pollRX(){
    if(this->serial.available() || true){
    std::string new_buff = this->serial.readline();
    this->parseBuff(new_buff);

    }else{
        RCLCPP_INFO(manager->get_logger(), "Msg too small to parse? %s", this->cbs_id.c_str());
    }
};

void CBSDevice::parseBuff(std::string buff){
    //! need switch case to figure out msg type from id
    rover_msgs::msg::ArmPanel arm_panel_msg;
    if(sscanf(buff.c_str(), "$arm_joy(%hi,%hi,%hi,%i,%hi,%i,%i,%i)\n", &arm_panel_msg.left.y, &arm_panel_msg.left.meow, &arm_panel_msg.left.z, &arm_panel_msg.left.button, &arm_panel_msg.right.meow, &arm_panel_msg.right.y, &arm_panel_msg.right.z, &arm_panel_msg.right.button) == 8){
        manager->arm_panel_publisher->publish(arm_panel_msg);
        RCLCPP_INFO(manager->get_logger(), "Msg Published: %s", buff.c_str());
    };
}

