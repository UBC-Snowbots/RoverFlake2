// Regression + characterization test for the driver's conf-get path
// (MoteusDriverNode::readFlashConfig) against a scripted fdcanusb emulator on
// a pty — no hardware, no ROS graph.  Spawns test/fdcanusb_emulator.py per
// scenario; EMULATOR_PATH is injected by CMake.
//
// The fault scenarios are CHARACTERIZATION: they assert what the vendored
// moteus client actually does today (silent 0.0 on timeout, stale-line
// poisoning, one-register shift), not what correct behavior would be.

#include <gtest/gtest.h>

#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "moteus.h"
#include <rover_arm_common/motor_config.h>

namespace mot = mjbots::moteus;

namespace {

// Values scripted in fdcanusb_emulator.py — keep the two tables in sync.
const std::map<std::string, float> kScripted = {
    {"servo.default_accel_limit",    2.0f},
    {"servo.default_velocity_limit", 0.15f},
    {"servo.max_velocity",           0.35f},
    {"servopos.position_min",        -0.45f},
    {"servopos.position_max",        0.55f},
    {"servo.max_current_A",          9.0f},
    {"servo.pid_position.kp",        17000.0f},
    {"servo.pid_position.ki",        123.0f},
    {"servo.pid_position.kd",        3500.0f},
    {"servo.max_voltage",            30.0f},
    {"servo.max_power_W",            250.0f},
    {"servo.default_timeout_s",      0.8f},
};

// Register order exactly as the driver iterates it (readFlashConfig loops
// configs_[m].get_configs()) — names only, defaults don't matter.
std::vector<std::string> RegNames() {
    std::vector<std::string> names;
    for (const auto& [reg, val] : MotorConfig{}.get_configs()) names.push_back(reg);
    return names;
}

// Spawns the pty emulator, reads the slave path it prints, kills it on teardown.
class EmulatorProc {
 public:
    explicit EmulatorProc(const std::vector<std::string>& extra_args) {
        int fds[2];
        if (::pipe(fds) != 0) return;
        pid_ = ::fork();
        if (pid_ == 0) {
            ::dup2(fds[1], STDOUT_FILENO);
            ::close(fds[0]); ::close(fds[1]);
            std::vector<std::string> args = {"python3", EMULATOR_PATH};
            args.insert(args.end(), extra_args.begin(), extra_args.end());
            std::vector<char*> argv;
            for (auto& a : args) argv.push_back(const_cast<char*>(a.c_str()));
            argv.push_back(nullptr);
            ::execvp("python3", argv.data());
            ::_exit(127);
        }
        ::close(fds[1]);
        char buf[256] = {};
        ssize_t pos = 0;
        while (pos < (ssize_t)sizeof(buf) - 1) {  // read the pty path line
            const ssize_t n = ::read(fds[0], &buf[pos], 1);
            if (n <= 0 || buf[pos] == '\n') { buf[pos] = 0; break; }
            pos++;
        }
        ::close(fds[0]);
        pty_path_ = buf;
    }
    ~EmulatorProc() {
        if (pid_ > 0) { ::kill(pid_, SIGTERM); ::waitpid(pid_, nullptr, 0); }
    }
    const std::string& pty_path() const { return pty_path_; }

 private:
    pid_t pid_ = -1;
    std::string pty_path_;
};

struct ReadResult {
    bool ok = false;                 // what readFlashConfig would return
    std::map<std::string, float> vals;
};

// Faithful copy of readFlashConfig's bus sequence (moteus_driver_node.cpp:952):
// SetQuery liveness gate, one DiagnosticFlush, then per-register
// DiagnosticCommand("conf get ...", kExpectSingleLine) parsed with strtof.
// Run inline instead of behind guardTransport's watchdog thread so failures
// surface as test results rather than abandoned transports.
ReadResult ReplicateReadFlashConfig(const std::string& pty_path) {
    ReadResult r;
    // The driver opens via Fdcanusb(path), whose TIOCGSERIAL ioctl a pty does
    // not support — open ourselves and use the lib's unit-test fd constructor;
    // everything from framing down is the identical code path.
    const int fd = ::open(pty_path.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) return r;
    termios t{};
    ::tcgetattr(fd, &t);
    ::cfmakeraw(&t);
    ::tcsetattr(fd, TCSANOW, &t);
    auto transport = std::make_shared<mot::Fdcanusb>(fd, fd);  // owns fd

    mot::Controller::Options opts;   // mirrors reInitTransport()
    opts.id = 5;
    opts.transport = transport;
    opts.query_format.q_current = mot::kFloat;
    opts.query_format.power     = mot::kFloat;
    opts.query_format.aux2_gpio = mot::kInt8;
    opts.position_format.velocity_limit = mot::kFloat;
    opts.position_format.accel_limit    = mot::kFloat;
    auto ctrl = std::make_shared<mot::Controller>(opts);

    if (!ctrl->SetQuery()) return r;   // liveness gate
    ctrl->DiagnosticFlush();           // driver: "drop stale lines or every reply shifts"
    for (const auto& reg : RegNames()) {
        const std::string raw = ctrl->DiagnosticCommand(
            "conf get " + reg, mot::Controller::kExpectSingleLine);
        r.vals[reg] = std::strtof(raw.c_str(), nullptr);
    }
    r.ok = true;   // the driver's guard succeeded and out is non-empty
    return r;
}

ReadResult RunScenario(const std::vector<std::string>& emulator_args) {
    EmulatorProc emu(emulator_args);
    EXPECT_FALSE(emu.pty_path().empty()) << "emulator failed to start";
    if (emu.pty_path().empty()) return {};
    return ReplicateReadFlashConfig(emu.pty_path());
}

}  // namespace

// Regression proper: a healthy motor's tuned values must arrive untouched.
TEST(ConfGetPath, CleanReadMatchesScriptedValues) {
    const auto r = RunScenario({"--mode", "clean"});
    ASSERT_TRUE(r.ok);
    for (const auto& [reg, want] : kScripted)
        EXPECT_EQ(want, r.vals.at(reg)) << reg;
}

// CHARACTERIZATION — one stale line in the device buffer after the flush:
// kExpectSingleLine takes the first line unconditionally, so register #1 reads
// the stale 999.0; its real reply drains in the same 48-byte poll and dies
// with the per-command context, so later registers are NOT shifted.
TEST(ConfGetPath, StaleLinePoisonsFirstRegisterOnly) {
    const auto r = RunScenario({"--mode", "stale"});
    ASSERT_TRUE(r.ok);   // driver still reports success — corruption is silent
    const auto regs = RegNames();
    EXPECT_EQ(999.0f, r.vals.at(regs[0])) << "stale line adopted as " << regs[0];
    for (size_t i = 1; i < regs.size(); i++)
        EXPECT_EQ(kScripted.at(regs[i]), r.vals.at(regs[i])) << regs[i];
}

// CHARACTERIZATION — reply times out (5 empty polls), firmware answers late:
// DiagnosticCommand swallows ETIMEDOUT and returns "", strtof turns it into
// 0.0 (kp reads "untuned").  The late line is then adopted by the NEXT
// register; with a fast device the next register's own reply arrives in the
// same poll and is discarded, so the shift stops there.
TEST(ConfGetPath, DroppedReplyReadsZeroThenPoisonsNext) {
    const auto r = RunScenario({"--mode", "drop", "--drop-index", "7"});
    ASSERT_TRUE(r.ok);   // silent again
    const auto regs = RegNames();  // regs[6] = kp, regs[7] = ki
    EXPECT_EQ(0.0f, r.vals.at(regs[6])) << "timeout -> empty string -> 0.0";
    EXPECT_EQ(kScripted.at(regs[6]), r.vals.at(regs[7])) << "ki adopts kp's late line";
    for (size_t i = 0; i < regs.size(); i++) {
        if (i == 6 || i == 7) continue;
        EXPECT_EQ(kScripted.at(regs[i]), r.vals.at(regs[i])) << regs[i];
    }
}

// CHARACTERIZATION — same timeout, but the firmware always answers one poll
// late (borderline latency, the regime that caused the timeout in the first
// place): every register after kp reads its PREDECESSOR's value for the rest
// of the loop.  Plausible-but-wrong numbers, exactly the HMI symptom.
TEST(ConfGetPath, DroppedReplyWithLatencyShiftsAllSubsequent) {
    const auto r = RunScenario({"--mode", "drop", "--drop-index", "7", "--lag"});
    ASSERT_TRUE(r.ok);   // and still reported as a good read
    const auto regs = RegNames();
    for (size_t i = 0; i < 6; i++)
        EXPECT_EQ(kScripted.at(regs[i]), r.vals.at(regs[i])) << regs[i];
    EXPECT_EQ(0.0f, r.vals.at(regs[6])) << "kp: timeout -> 0.0";
    for (size_t i = 7; i < regs.size(); i++)
        EXPECT_EQ(kScripted.at(regs[i - 1]), r.vals.at(regs[i]))
            << regs[i] << " shows " << regs[i - 1] << "'s value";
}
