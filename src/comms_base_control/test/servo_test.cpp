#include <catch2/catch_all.hpp>
#include <servo.hpp>

#include <cmath>
#include <utility>
#include <vector>

// Records what servo.cpp would have sent to pigpiod.
static std::vector<std::pair<unsigned, unsigned>> sent;  // (gpio, pulse_us)

extern "C" int set_servo_pulsewidth(int, unsigned user_gpio, unsigned pulsewidth) {
    sent.emplace_back(user_gpio, pulsewidth);
    return 0;
}

static ServoType latchType() {
    ServoType type;
    type.name = "latch";
    type.min_us = 500;
    type.max_us = 1750;
    type.open_us = 1700;
    type.close_us = 600;
    return type;
}

TEST_CASE("servo open/close send the configured pulse to its pin")
{
    sent.clear();
    Servo servo("claw", 27, latchType(), 0);

    REQUIRE(servo.open());
    REQUIRE(sent.back() == std::make_pair(27u, 1700u));
    REQUIRE(servo.close());
    REQUIRE(sent.back() == std::make_pair(27u, 600u));
    REQUIRE(servo.release());
    REQUIRE(sent.back() == std::make_pair(27u, 0u));
}

TEST_CASE("servo pulse widths stay within the type's limits")
{
    sent.clear();
    Servo servo("claw", 27, latchType(), 0);

    REQUIRE(servo.setPulseUs(9999));
    REQUIRE(sent.back().second == 1750);
    REQUIRE(servo.setPulseUs(10));
    REQUIRE(sent.back().second == 500);

    SECTION("non-finite pulse widths are refused without touching the pin")
    {
        sent.clear();
        REQUIRE_FALSE(servo.setPulseUs(std::nanf("")));
        REQUIRE(sent.empty());
    }
}

TEST_CASE("servo without a pigpiod connection refuses to move")
{
    sent.clear();
    Servo servo("claw", 27, latchType(), -1);

    REQUIRE_FALSE(servo.open());
    REQUIRE(sent.empty());
}

TEST_CASE("servo type validation rejects non-finite calibration")
{
    ServoType type = latchType();
    REQUIRE_NOTHROW(type.validate());

    type.open_us = std::nanf("");
    REQUIRE_THROWS(type.validate());

    type = latchType();
    type.min_us = INFINITY;
    REQUIRE_THROWS(type.validate());
}
