// hmi_router — maps CBS panel button presses to HMI layout switches.
// Rising-edge detect on /cbs/left_panel_a buttons; button i publishes
// button_layouts[i] on /hmi/load_layout. Empty/missing entry = unbound.

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/generic_panel.hpp"
#include "std_msgs/msg/string.hpp"

class HmiRouter : public rclcpp::Node {
public:
    HmiRouter() : Node("hmi_router") {
        button_layouts_ = declare_parameter<std::vector<std::string>>(
            "button_layouts", std::vector<std::string>{});
        layout_pub_ = create_publisher<std_msgs::msg::String>("/hmi/load_layout", 10);
        panel_sub_  = create_subscription<rover_msgs::msg::GenericPanel>(
            "/cbs/left_panel_a", 10,
            std::bind(&HmiRouter::panelCallback, this, std::placeholders::_1));
    }

private:
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
            std_msgs::msg::String out;
            out.data = button_layouts_[i];
            layout_pub_->publish(out);
            RCLCPP_INFO(get_logger(), "button %zu → layout '%s'", i, out.data.c_str());
        }
    }

    std::vector<std::string> button_layouts_;
    std::vector<int16_t> prev_buttons_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr layout_pub_;
    rclcpp::Subscription<rover_msgs::msg::GenericPanel>::SharedPtr panel_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HmiRouter>());
    rclcpp::shutdown();
    return 0;
}
