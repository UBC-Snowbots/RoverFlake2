// Pure-math tests for the axis 5/6 differential wrist mapping
// (include/axis_5_6_differential.h). Bench convention 2026-08-03:
// A5 -> both motors same direction; A6 -> motors opposed.
#include <gtest/gtest.h>
#include "axis_5_6_differential.h"

namespace {
const float kGrid[] = {-10.f, -3.5f, -1.f, -0.25f, 0.f, 0.25f, 1.f, 2.75f, 10.f};
}

TEST(WristMath, Axis5OnlyDrivesBothSameDirection) {
  for (float v : kGrid) {
    float m5 = 0.f, m6 = 0.f;
    differential_drive(v, 0.f, m5, m6);
    EXPECT_FLOAT_EQ(m5, v) << "a5=" << v;
    EXPECT_FLOAT_EQ(m6, v) << "a5=" << v;
  }
}

TEST(WristMath, Axis6OnlyDrivesMotorsOpposed) {
  for (float v : kGrid) {
    float m5 = 0.f, m6 = 0.f;
    differential_drive(0.f, v, m5, m6);
    EXPECT_FLOAT_EQ(m5, v) << "a6=" << v;
    EXPECT_FLOAT_EQ(m6, -v) << "a6=" << v;
  }
}

TEST(WristMath, InverseRoundTripsJointSpace) {
  for (float a5 : kGrid) {
    for (float a6 : kGrid) {
      float m5, m6, r5, r6;
      differential_drive(a5, a6, m5, m6);
      differential_drive_inverse(m5, m6, r5, r6);
      EXPECT_FLOAT_EQ(r5, a5) << "a5=" << a5 << " a6=" << a6;
      EXPECT_FLOAT_EQ(r6, a6) << "a5=" << a5 << " a6=" << a6;
    }
  }
}

TEST(WristMath, ForwardRoundTripsMotorSpace) {
  for (float m5 : kGrid) {
    for (float m6 : kGrid) {
      float a5, a6, r5, r6;
      differential_drive_inverse(m5, m6, a5, a6);
      differential_drive(a5, a6, r5, r6);
      EXPECT_FLOAT_EQ(r5, m5) << "m5=" << m5 << " m6=" << m6;
      EXPECT_FLOAT_EQ(r6, m6) << "m5=" << m5 << " m6=" << m6;
    }
  }
}
