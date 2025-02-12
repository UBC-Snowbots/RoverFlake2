#include "cbsDevice.h"
#include "cbsManagerNode.h"
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
            serial.readline();


        } catch(...){
    RCLCPP_INFO(manager->get_logger(), "..%s Port Open Failure: %s No port name given. Aborting serial connection.", ConsoleFormat::red(), ConsoleFormat::reset());
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


