#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <SFML/Audio.hpp>
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>

class SpeakerNode : public rclcpp::Node
{
public:
    SpeakerNode() : Node("speaker_node")
    {
    }
    void init(){
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    qos.keep_last(10);
                aux_sub = this->create_subscription<std_msgs::msg::String>(
            "speaker/command", qos, std::bind(&SpeakerNode::commandCallback, this, std::placeholders::_1)); 

    }

    void playSound(const std::string& soundFileName) {
        sf::SoundBuffer buffer;

        if (!buffer.loadFromFile(sound_path + soundFileName)) {
            std::cerr << "Failed to load sound file: " << soundFileName << std::endl;
            return;
        }else {
            std::cerr << "Load Success: " << soundFileName << std::endl;
          
        }


        speaker.setBuffer(buffer);
        speaker.setVolume(100);
        speaker.play();
        while(speaker.getStatus() == sf::Sound::Playing){
             sf::sleep(sf::milliseconds(10));
         }
       
  
    }

private:
    sf::Sound speaker;
    sf::SoundBuffer sfxBuffer;
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("rover_sounds");
    std::string sound_path = package_share_directory + "/sounds/";
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr aux_sub;

void commandCallback(const std_msgs::msg::String::ConstSharedPtr &cmd){
    playSound(cmd->data);
}
};