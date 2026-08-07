// motor_status_display_test.cpp — offscreen pin of the Motor Telemetry debug
// decode: synthetic MoteusArmStatus in, Mode/Fault cell text out. Expected
// strings encode moteus firmware truth (fw/bldc_servo_structs.h modes,
// fw/error.h limit codes 96-104; hard faults 1-48 render as bare 'ERR n').
// Run with QT_QPA_PLATFORM=offscreen (main() sets it if unset).

#include <gtest/gtest.h>
#include <QApplication>
#include <QLabel>
#include <rover_hmi_core/arm/motor_status_module.h>
#include <rover_arm_common/motor_addressing.h>
#include <cstdlib>
#include <memory>

// Test seam — friend of MotorStatusModule (declared in its header).
struct MotorStatusTestAccess {
    static void feed(MotorStatusModule& m, rover_msgs::msg::MoteusArmStatus msg) {
        m.onFeedback(std::make_shared<rover_msgs::msg::MoteusArmStatus>(std::move(msg)));
    }
    static QLabel* cell(MotorStatusModule& m, int r, int c) { return m.cells_[r][c]; }
};
using Peer = MotorStatusTestAccess;

namespace {

// Mirror enum Col in motor_status_module.cpp.
constexpr int COL_MODE = 0, COL_FAULT = 1;

rover_msgs::msg::MoteusArmStatus makeStatus(int mode, int fault, bool connected = true) {
    rover_msgs::msg::MoteusArmStatus msg;
    msg.status.resize(NUM_MOTORS);
    msg.limit_switches.resize(NUM_MOTORS, false);
    msg.can_device = "/dev/fdcanusb";
    msg.motors_replying = NUM_MOTORS;
    for (auto& s : msg.status) { s.connected = true; s.moteus_mode = 10; s.moteus_fault = 0; }
    msg.status[0].moteus_mode = mode;
    msg.status[0].moteus_fault = fault;
    msg.status[0].connected = connected;
    return msg;
}

} // namespace

class MotorStatusDisplay : public ::testing::Test {
protected:
    void SetUp() override {
        w_ = mod_.createWidget(nullptr);
        ASSERT_NE(w_, nullptr);
    }
    void TearDown() override { delete w_; }

    std::string modeText(int mode, int fault = 0, bool connected = true) {
        Peer::feed(mod_, makeStatus(mode, fault, connected));
        return Peer::cell(mod_, 0, COL_MODE)->text().toStdString();
    }
    std::string faultText(int fault, int mode = 10, bool connected = true) {
        Peer::feed(mod_, makeStatus(mode, fault, connected));
        return Peer::cell(mod_, 0, COL_FAULT)->text().toStdString();
    }

    MotorStatusModule mod_;
    QWidget* w_ = nullptr;
};

TEST_F(MotorStatusDisplay, KnownModesDecodeToFirmwareNames) {
    static const struct { int mode; const char* name; } CASES[] = {
        {0, "Stopped"}, {1, "FAULT"}, {2, "Enabling"}, {3, "Calibrating"},
        {4, "Cal Complete"}, {5, "PWM"}, {6, "Voltage"}, {7, "VoltageFOC"},
        {8, "VoltageDQ"}, {9, "Current"}, {10, "Position"}, {11, "TIMEOUT"},
        {12, "ZeroVelocity"}, {13, "Within"}, {14, "MeasureInd"}, {15, "Brake"},
    };
    for (const auto& c : CASES)
        EXPECT_EQ(modeText(c.mode), c.name) << "mode " << c.mode;
}

TEST_F(MotorStatusDisplay, UndecodedModesFallBackToNumber) {
    // Anything beyond the firmware's 0-15 range pins to the numeric fallback.
    EXPECT_EQ(modeText(16), "Mode 16");
    EXPECT_EQ(modeText(42), "Mode 42");
}

TEST_F(MotorStatusDisplay, FaultZeroShowsOk) {
    EXPECT_EQ(faultText(0), "OK");
}

TEST_F(MotorStatusDisplay, HardFaultsShowErrNumber) {
    // Latched faults (fw/error.h: 1-7, 32-48) have no name table — bare code.
    EXPECT_EQ(faultText(32, /*mode=*/1), "ERR 32");
    EXPECT_EQ(faultText(1, 1), "ERR 1");
    EXPECT_EQ(faultText(48, 1), "ERR 48");
}

TEST_F(MotorStatusDisplay, LimitCodesDecodeToLabels) {
    static const struct { int code; const char* label; } CASES[] = {
        {96, "VEL CAP"}, {97, "PWR CAP"}, {98, "VOLT CAP"}, {99, "CUR CAP"},
        {100, "FET TEMP"}, {101, "MTR TEMP"}, {102, "TRQ CAP"},
        {103, "POS BOUND"}, {104, "FLUX BRK"},
    };
    for (const auto& c : CASES)
        EXPECT_EQ(faultText(c.code), c.label) << "limit code " << c.code;
}

TEST_F(MotorStatusDisplay, UnknownLimitCodeFallsBackToLimNumber) {
    EXPECT_EQ(faultText(105), "LIM 105");
}

TEST_F(MotorStatusDisplay, DisconnectedShowsNoReply) {
    EXPECT_EQ(modeText(10, 0, /*connected=*/false), "NO REPLY");
    // Fault cell keeps decoding last-known values on a stale row (current
    // behavior — only the styling marks staleness).
    EXPECT_EQ(faultText(0, 10, false), "OK");
}

int main(int argc, char** argv) {
    setenv("QT_QPA_PLATFORM", "offscreen", 0);  // keep caller override
    QApplication app(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
