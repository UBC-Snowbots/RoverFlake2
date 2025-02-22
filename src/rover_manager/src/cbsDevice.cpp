#include "cbsDevice.h"
#include "cbsHardwareManagerNode.h"
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

int CBSDevice::findMyPort(){
    if(serial.isOpen()){
        if(port_found){
            return SERIAL_PORT_ALREADY_FOUND;
        }else{
            return SERIAL_PORT_BUSY_OR_DNE;
        }
    }else{
        int current_port_index = 0;
        serial.setBaudrate(baudrate);
        while(!port_found){
        serial.setPort(manager->possible_ports[current_port_index]);
        try{serial.open();
            
            rclcpp::Clock clock(RCL_SYSTEM_TIME);
            rclcpp::Time start_time = clock.now();
            serial.flush();
            while(serial.available() == 0){
                //wait
                if(clock.now().seconds() - start_time.seconds() > 3){
                    // SERIAL_PORT_TIMEOUT;
                    throw 1;
                }

            }
            std::string buff;
            bool id_located = false;
            int garbage_lines = 0;
            while(!id_located){
                buff = "";
                serial.readline(buff);
                size_t start = buff.find("$") +1;
                size_t end_of_id = buff.find("(");                    
                if(start != std::string::npos && end_of_id != std::string::npos){ //If we actually found the position of "$"
                    std::string found_panel_id = buff.substr(start, end_of_id - start); // Parse id from start of string to end
                    RCLCPP_INFO(manager->get_logger(), "%s %s %s | String ID found at %li: %s'%s'%s", yellow(), this->id.c_str(), reset(), start, green(), found_panel_id.c_str(), reset() );
                    if(found_panel_id.find(this->id) != std::string::npos){
                        this->port_path = manager->possible_ports[current_port_index];
                        RCLCPP_INFO(manager->get_logger(), "%s Thats my device! %s %s is Taking control of port: %s %s", bold(), green(), this->id.c_str(), this->port_path.c_str(), reset()); 
                        id_located = true;
                        ready_for_polling = true;
                        manager->taken_ports.push_back(port_path);
                        return PORT_FOUND_SUCCESS;
                    }else{
                        RCLCPP_INFO(manager->get_logger(), "%s Woops! This isn't my port. %s %s is disconnecting from port: %s. %s Gonna try a different one.", bold(), green(), this->id.c_str(), this->port_path.c_str(), reset()); 
                        throw 1;
                    }
                    // this->id = panel_id;
                }else{
                    RCLCPP_WARN(manager->get_logger(), "%s, Serial buff garbage line found", this->id.c_str());
                    garbage_lines++;
                    if(garbage_lines > 5000){
                        RCLCPP_INFO(manager->get_logger(), "Too much garbage. Trying a new port");
                        throw 1;
                    }
                }
                
            }


        } catch(...){
    RCLCPP_INFO(manager->get_logger(), "..%s Port Open Failure : %s %s Aborting serial connection and trying new port.", ConsoleFormat::yellow(), manager->possible_ports[current_port_index].c_str(), ConsoleFormat::reset());
        // return SERIAL_PORT_BUSY_OR_DNE;
        serial.close();
        current_port_index++;
        if(current_port_index +1 > manager->possible_ports.size()){
            return PORT_NOT_FOUND;
        }
            }


        }
    }
}


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
                    std::string found_panel_id = buff.substr(start, end_of_id - start); // Parse id from start of string to end
                    RCLCPP_INFO(manager->get_logger(), "%s %s %s | String ID found at %li: %s'%s'%s", yellow(), this->id.c_str(), reset(), start, green(), found_panel_id.c_str(), reset() );
                    id_located = true;
                    // this->id = panel_id;
                }else{
                    RCLCPP_WARN(manager->get_logger(), "%s, Serial buff garbage line found", this->id.c_str());
                }
                
            }


        } catch(...){
    RCLCPP_INFO(manager->get_logger(), "..%s Port Open Failure: %s No port name given or something is wrong. Aborting serial connection.", ConsoleFormat::red(), ConsoleFormat::reset());
        return SERIAL_PORT_BUSY_OR_DNE;
            }
    }

    
}

// void CBSDevice::setID(std::string my_id){
//     this->id = my_id;
//     // RCLCPP_INFO(manager->get_logger(), "Id set: %s", id.c_str());
// }

void CBSDevice::init(int baudrate_, std::string new_id, CBSHardwareManagerNode* manager_){
    this->manager = manager_;
    this->id = new_id;
    this->baudrate = baudrate_;
    RCLCPP_INFO(manager->get_logger(), "Logger attached. Hello world! im %s", this->id.c_str());

}
void CBSDevice::pollRX(){
    if(this->serial.available() > this->min_msg_size){
    std::string new_buff = this->serial.readline();
    this->parseBuff(new_buff);
    this->serial.flush();
    }else{
        RCLCPP_INFO(manager->get_logger(), "Msg too small to parse? %s", this->id.c_str());
    }
};

void CBSDevice::parseBuff(std::string buff){
    //! need switch case to figure out msg type from id
    rover_msgs::msg::ArmPanel arm_panel_msg;
    buff.erase(std::remove(buff.begin(), buff.end(), '\r'), buff.end());  // Remove '\r'
buff.erase(std::remove(buff.begin(), buff.end(), '\n'), buff.end());  // Remove '\n'

    // if(sscanf(buff.c_str(), "$arm_joy( %d , %d , %d , %d , %d , %d , %d , %d )", &arm_panel_msg.left.x, &arm_panel_msg.left.y, &arm_panel_msg.left.z, &arm_panel_msg.left.button, &arm_panel_msg.right.x, &arm_panel_msg.right.y, &arm_panel_msg.right.z, &arm_panel_msg.right.button) == 8){
    //     manager->arm_panel_publisher->publish(arm_panel_msg);
    //     //  RCLCPP_INFO(manager->get_logger(), "Msg Published: %s", buff.c_str());
    //     //RCLCPP_INFO(manager->get_logger(), "xLeft is %i", arm_panel_msg.left.x);
    // RCLCPP_INFO(manager->get_logger(), "Raw buffer: [%s]", buff.c_str());
    // } //! sscanf just won't read the x values. No idea why
// // for (char c : buff) {
// //     printf("Char: '%c' (ASCII: 0x%02X)\n", c, static_cast<unsigned char>(c));
// // }
//     }
if(buff.find(")") != std::string::npos){


    std::string data = buff;
std::replace(data.begin(), data.end(), '(', ' ');  // Replace '(' with space
std::replace(data.begin(), data.end(), ')', ' ');  // Replace ')' with space
std::replace(data.begin(), data.end(), ',', ' ');  // Replace ',' with space
    data.erase(0, data.find_first_not_of(" "));  // Trim left
        data.erase(data.find_last_not_of(" ") + 1);  // Trim right



std::stringstream ss(data);
std::string tag;
int values[8] = {-1};

ss >> tag;  // Skip "$arm_joy"
for (int i = 0; i < 8; i++) {
    if(!(ss >> values[i])){
   
        RCLCPP_ERROR(manager->get_logger(), "Parse failure on %s", this->id.c_str());
        return;
    }
}

arm_panel_msg.left.x = values[0];
arm_panel_msg.left.y = values[1];
arm_panel_msg.left.z = values[2];
arm_panel_msg.left.button = values[3];
arm_panel_msg.right.x = values[4];
arm_panel_msg.right.y = values[5];
arm_panel_msg.right.z = values[6];
arm_panel_msg.right.button = values[7];
        manager->arm_panel_publisher->publish(arm_panel_msg);

RCLCPP_INFO(manager->get_logger(), "Manually Parsed: xLeft=%d, xRight=%d", 
            arm_panel_msg.left.x, arm_panel_msg.right.x);
}
}

