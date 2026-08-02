// keyboard_servo_input.cpp — arrow-key cartesian EE jog for MoveIt Servo.
//
// Publishes unitless TwistStamped on /servo_node/delta_twist_cmds (same topic
// joy_arm_control uses), base_link frame.
//
// Key events only *latch* a direction; a 30 Hz timer streams the twist while
// the latch is fresh (refreshed by OS autorepeat) and stops ~200 ms after the
// key is released. This gives Servo the smooth command stream it expects —
// raw per-keystroke publishing makes the arm stutter at the autorepeat rate.
//
//   arrows     = +/-X, +/-Y        w / s = +Z / -Z        q = quit

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <chrono>
#include <thread>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

namespace {
constexpr char KEY_RIGHT = 0x43, KEY_LEFT = 0x44, KEY_UP = 0x41, KEY_DOWN = 0x42;
constexpr auto LATCH_TIMEOUT = std::chrono::milliseconds(200);
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string FRAME_ID    = "base_link";

struct termios g_cooked;
void restoreTerminal(int sig) {
    tcsetattr(0, TCSANOW, &g_cooked);
    rclcpp::shutdown();
    if (sig) exit(0);
}
} // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("keyboard_servo_input");
    auto pub = node->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, 10);

    std::atomic<double> vx{0}, vy{0}, vz{0};
    std::atomic<int64_t> last_key_ns{0};

    // 30 Hz streamer: publish the latched direction while it's fresh
    auto timer = node->create_wall_timer(std::chrono::milliseconds(33), [&]() {
        auto age = node->get_clock()->now().nanoseconds() - last_key_ns.load();
        if (age > std::chrono::nanoseconds(LATCH_TIMEOUT).count()) return;  // key released
        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = node->get_clock()->now();
        msg.header.frame_id = FRAME_ID;
        msg.twist.linear.x = vx.load();
        msg.twist.linear.y = vy.load();
        msg.twist.linear.z = vz.load();
        pub->publish(msg);
    });
    (void)timer;
    std::thread spinner([&]() { rclcpp::spin(node); });

    tcgetattr(0, &g_cooked);
    struct termios raw = g_cooked;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(0, TCSANOW, &raw);
    signal(SIGINT, restoreTerminal);

    puts("Keyboard EE jog — arrows: X/Y   w/s: Z up/down   q: quit");
    puts("(hold a key to move; motion stops ~0.2 s after release)");

    char c;
    while (rclcpp::ok()) {
        if (read(0, &c, 1) < 0) break;

        double x = 0, y = 0, z = 0;
        if      (c == KEY_UP)    x =  1.0;   // arrow escape codes: ESC [ A/B/C/D —
        else if (c == KEY_DOWN)  x = -1.0;   // ESC and '[' fall through harmlessly
        else if (c == KEY_RIGHT) y = -1.0;
        else if (c == KEY_LEFT)  y =  1.0;
        else if (c == 'w')       z =  1.0;
        else if (c == 's')       z = -1.0;
        else if (c == 'q')       break;
        else continue;

        vx = x; vy = y; vz = z;
        last_key_ns = node->get_clock()->now().nanoseconds();
    }

    restoreTerminal(0);
    spinner.detach();
    return 0;
}
