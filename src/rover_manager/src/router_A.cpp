// hmi_router — maps CBS panel button presses to HMI layout switches.
// Rising-edge detect on /cbs/left_panel_a buttons. button_layouts[i] is either
// a bare layout name (published on /hmi/load_layout, all windows) or a scene:
// comma-separated "instance:layout" pairs, each published on
// /hmi/<instance>/load_layout. Empty/missing entry = unbound.

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/generic_panel.hpp"
#include "std_msgs/msg/string.hpp"

#include <map>
#include <sstream>

class HmiRouter : public rclcpp::Node {
public:
    HmiRouter() : Node("hmi_router") {
        button_layouts_ = declare_parameter<std::vector<std::string>>(
            "button_layouts", std::vector<std::string>{});
        panel_sub_ = create_subscription<rover_msgs::msg::GenericPanel>(
            "/cbs/left_panel_a", 10,
            std::bind(&HmiRouter::panelCallback, this, std::placeholders::_1));
    }

private:
    static std::string trim(const std::string& s) {
        size_t a = s.find_first_not_of(" \t");
        if (a == std::string::npos) return "";
        size_t b = s.find_last_not_of(" \t");
        return s.substr(a, b - a + 1);
    }

    // "left:camera, center:drive" → {{"/hmi/left/load_layout","camera"}, …}
    // "arm" → {{"/hmi/load_layout","arm"}}. Malformed pairs are WARNed and skipped.
    std::vector<std::pair<std::string, std::string>> parseSceneEntry(const std::string& entry) {
        std::vector<std::pair<std::string, std::string>> out;
        std::stringstream ss(entry);
        std::string token;
        while (std::getline(ss, token, ',')) {
            token = trim(token);
            if (token.empty()) continue;
            size_t colon = token.find(':');
            if (colon == std::string::npos) {
                out.emplace_back("/hmi/load_layout", token);
                continue;
            }
            std::string inst = trim(token.substr(0, colon));
            std::string layout = trim(token.substr(colon + 1));
            if (inst.empty() || layout.empty()) {
                RCLCPP_WARN(get_logger(), "malformed scene pair '%s' — skipped", token.c_str());
                continue;
            }
            out.emplace_back("/hmi/" + inst + "/load_layout", layout);
        }
        return out;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubFor(const std::string& topic) {
        auto& pub = pubs_[topic];
        if (!pub) pub = create_publisher<std_msgs::msg::String>(topic, 10);
        return pub;
    }

    void panelCallback(const rover_msgs::msg::GenericPanel::SharedPtr msg) {
        // First message (or size change) only sets the baseline — a button
        // already held at startup must not fire.
        if (prev_buttons_.size() != msg->buttons.size()) {
            prev_buttons_.assign(msg->buttons.begin(), msg->buttons.end());
            return;
        }
        for (size_t i = 0; i < msg->buttons.size(); ++i) {
            bool rising = msg->buttons[i] && !prev_buttons_[i];
            prev_buttons_[i] = msg->buttons[i];
            if (!rising || i >= button_layouts_.size() || button_layouts_[i].empty())
                continue;
            for (const auto& [topic, layout] : parseSceneEntry(button_layouts_[i])) {
                std_msgs::msg::String out;
                out.data = layout;
                pubFor(topic)->publish(out);
                RCLCPP_INFO(get_logger(), "button %zu → %s '%s'", i, topic.c_str(), layout.c_str());
            }
        }
    }

    std::vector<std::string> button_layouts_;
    std::vector<int16_t> prev_buttons_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> pubs_;
    rclcpp::Subscription<rover_msgs::msg::GenericPanel>::SharedPtr panel_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HmiRouter>());
    rclcpp::shutdown();
    return 0;
}
