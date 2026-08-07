// motor_config_display_test.cpp — offscreen regression test for the Motor
// Params "Actual" column: synthetic MoteusArmStatus in, table label text out.
// Run with QT_QPA_PLATFORM=offscreen (main() sets it if unset).

#include <gtest/gtest.h>
#include <QApplication>
#include <rover_hmi_core/arm/motor_config_module.h>
#include <rover_arm_common/motor_addressing.h>
#include <cmath>
#include <cstdlib>
#include <memory>

// Test seam — friend of MotorConfigModule (declared in its header).
struct MotorConfigTestAccess {
    static void feed(MotorConfigModule& m, rover_msgs::msg::MoteusArmStatus msg) {
        m.onFeedback(std::make_shared<rover_msgs::msg::MoteusArmStatus>(std::move(msg)));
    }
    static void refresh(MotorConfigModule& m) { m.refreshDisplay(); }
    static QComboBox* sel(MotorConfigModule& m) { return m.motor_sel_; }
    static QLabel* actual(MotorConfigModule& m, int r) { return m.actuals_[r]; }
};
using Peer = MotorConfigTestAccess;

namespace {

constexpr int NAN_MOTOR = 3;  // this motor's config stays all-NaN → every cell '?'

// Distinct injected value per (motor, display row): row block + motor offset.
float inj(int m, int r) { return 1000.0f * (r + 1) + m; }

// Field order mirrors REGS[] rows in motor_config_module.cpp.
void fillConfig(rover_msgs::msg::BldcServoConfig& c, int m) {
    c.kp = inj(m, 0);  c.ki = inj(m, 1);  c.kd = inj(m, 2);
    c.max_current_amps = inj(m, 3);
    c.max_velocity = inj(m, 4);       c.max_acceleration = inj(m, 5);
    c.min_position = inj(m, 6);       c.max_position = inj(m, 7);
    c.max_voltage_volts = inj(m, 8);  c.max_power_watts = inj(m, 9);
    c.cmd_timeout_s = inj(m, 10);     c.gear_reduction = inj(m, 11);
}

rover_msgs::msg::MoteusArmStatus makeStatus() {
    rover_msgs::msg::MoteusArmStatus msg;
    msg.config.resize(NUM_MOTORS);
    msg.status.resize(NUM_MOTORS);
    msg.limit_switches.resize(NUM_MOTORS);
    for (int m = 0; m < NUM_MOTORS; m++) {
        auto& c = msg.config[m];
        if (m == NAN_MOTOR) {
            c.kp = c.ki = c.kd = c.max_current_amps = c.max_velocity =
            c.max_acceleration = c.min_position = c.max_position =
            c.max_voltage_volts = c.max_power_watts = c.cmd_timeout_s =
            c.gear_reduction = NAN;
        } else {
            fillConfig(c, m);
        }
    }
    return msg;
}

} // namespace

TEST(MotorConfigDisplay, ActualColumnShowsInjectedValuesPerMotor) {
    MotorConfigModule mod;
    QWidget* w = mod.createWidget(nullptr);
    ASSERT_NE(w, nullptr);
    Peer::feed(mod, makeStatus());

    for (int m = 0; m < NUM_MOTORS; m++) {
        Peer::sel(mod)->setCurrentIndex(m);  // signal-driven refresh (except m==0)
        Peer::refresh(mod);                  // explicit, covers the m==0 no-emit case
        for (int r = 0; r < MotorConfigModule::NUM_REGS; r++) {
            const QString expect = (m == NAN_MOTOR)
                ? QStringLiteral("?") : MotorConfigModule::fmtVal(inj(m, r));
            EXPECT_EQ(Peer::actual(mod, r)->text().toStdString(), expect.toStdString())
                << "motor " << m << " row " << r;
        }
    }
    delete w;
}

TEST(MotorConfigDisplay, SwitchingMotorSwapsRows) {
    MotorConfigModule mod;
    QWidget* w = mod.createWidget(nullptr);
    Peer::feed(mod, makeStatus());

    Peer::sel(mod)->setCurrentIndex(1);
    const QString kp_m1 = Peer::actual(mod, 0)->text();
    Peer::sel(mod)->setCurrentIndex(5);
    const QString kp_m5 = Peer::actual(mod, 0)->text();
    EXPECT_EQ(kp_m1.toStdString(), MotorConfigModule::fmtVal(inj(1, 0)).toStdString());
    EXPECT_EQ(kp_m5.toStdString(), MotorConfigModule::fmtVal(inj(5, 0)).toStdString());
    EXPECT_NE(kp_m1.toStdString(), kp_m5.toStdString());
    delete w;
}

TEST(MotorConfigDisplay, NoMessageShowsUnread) {
    MotorConfigModule mod;
    QWidget* w = mod.createWidget(nullptr);
    Peer::refresh(mod);
    for (int r = 0; r < MotorConfigModule::NUM_REGS; r++)
        EXPECT_EQ(Peer::actual(mod, r)->text().toStdString(), "?") << "row " << r;
    delete w;
}

int main(int argc, char** argv) {
    setenv("QT_QPA_PLATFORM", "offscreen", 0);               // keep caller override
    setenv("ROVERFLAKE_ROOT", "/nonexistent_test_root", 1);  // hermetic arm_params.ini
    QApplication app(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
