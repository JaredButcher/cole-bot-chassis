// SPDX-License-Identifier: GPL-3.0-or-later
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "colebot/drive_math.h"

namespace cb {
namespace {

TEST(ErpmToWheelSpeedTest, ConvertsWithPolesAndReduction) {
  // 7000 eRPM, 14 magnets -> 1000 motor RPM -> 104.72 rad/s at 1:1.
  EXPECT_NEAR(erpmToWheelSpeed(7000, 14, 1.0f), 104.7198f, 1e-3f);
  // 12 magnets and a 4:1 reduction.
  EXPECT_NEAR(erpmToWheelSpeed(6000, 12, 4.0f), 1000.0f / 4.0f * 2.0f * 3.14159265f / 60.0f, 1e-3f);
  EXPECT_EQ(erpmToWheelSpeed(0, 14, 1.0f), 0.0f);
}

TEST(ErpmToWheelSpeedTest, InvalidParamsGiveZero) {
  EXPECT_EQ(erpmToWheelSpeed(7000, 0, 1.0f), 0.0f);
  EXPECT_EQ(erpmToWheelSpeed(7000, 14, 0.0f), 0.0f);
  EXPECT_EQ(erpmToWheelSpeed(7000, 14, std::numeric_limits<float>::quiet_NaN()), 0.0f);
}

TEST(DirectionTrackerTest, StartsForwardAndFollowsCommandAtRest) {
  DirectionTracker tracker;
  EXPECT_EQ(tracker.direction(), 1);
  EXPECT_EQ(tracker.update(-1, 0.0f, 2.0f), -1);
  EXPECT_EQ(tracker.update(1, 0.5f, 2.0f), 1);
}

TEST(DirectionTrackerTest, HoldsDirectionUntilSlowThroughReversal) {
  DirectionTracker tracker;
  tracker.update(1, 50.0f, 2.0f);
  EXPECT_EQ(tracker.update(-1, 40.0f, 2.0f), 1);  // still spinning forward, slowing
  EXPECT_EQ(tracker.update(-1, 5.0f, 2.0f), 1);
  EXPECT_EQ(tracker.update(-1, 1.0f, 2.0f), -1);  // below the flip speed: reversed
  EXPECT_EQ(tracker.update(-1, 30.0f, 2.0f), -1);
}

TEST(DirectionTrackerTest, StopCommandKeepsDirection) {
  DirectionTracker tracker;
  tracker.update(-1, 0.0f, 2.0f);
  EXPECT_EQ(tracker.update(0, 0.0f, 2.0f), -1);
}

TEST(PiControllerTest, PureFeedforward) {
  PiController loop;
  const VelocityLoopGains gains{0.01f, 0.05f, 0.0f, 0.0f, 0.0f};
  EXPECT_NEAR(loop.update(50.0f, 0.0f, 0.001f, gains), 0.55f, 1e-6f);
  EXPECT_NEAR(loop.update(-50.0f, 0.0f, 0.001f, gains), -0.55f, 1e-6f);
}

TEST(PiControllerTest, ZeroNanAndBelowMinSpeedStopAndReset) {
  PiController loop;
  const VelocityLoopGains gains{0.01f, 0.0f, 0.0f, 0.1f, 3.0f};
  loop.update(10.0f, 0.0f, 0.1f, gains);
  EXPECT_GT(loop.integral(), 0.0f);
  EXPECT_EQ(loop.update(2.0f, 0.0f, 0.1f, gains), 0.0f);  // below min_speed
  EXPECT_EQ(loop.integral(), 0.0f);
  EXPECT_EQ(loop.update(0.0f, 5.0f, 0.1f, gains), 0.0f);
  EXPECT_EQ(loop.update(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.1f, gains), 0.0f);
}

TEST(PiControllerTest, ProportionalAndIntegralAct) {
  PiController loop;
  const VelocityLoopGains gains{0.0f, 0.0f, 0.01f, 0.1f, 0.0f};
  // error 10, dt 0.1: p = 0.1, integral = 1.0 -> i = 0.1
  EXPECT_NEAR(loop.update(10.0f, 0.0f, 0.1f, gains), 0.2f, 1e-6f);
  EXPECT_NEAR(loop.integral(), 1.0f, 1e-6f);
}

TEST(PiControllerTest, OutputClampedAndIntegratorDoesNotWindUp) {
  PiController loop;
  const VelocityLoopGains gains{0.02f, 0.0f, 0.0f, 1.0f, 0.0f};
  for (int i = 0; i < 1000; ++i) {
    EXPECT_LE(loop.update(100.0f, 0.0f, 0.01f, gains), 1.0f);  // ff alone saturates
  }
  EXPECT_EQ(loop.integral(), 0.0f);  // held while saturated in the same direction
  // Overshoot: the error reverses, so the integrator may move again.
  loop.update(100.0f, 150.0f, 0.01f, gains);
  EXPECT_LT(loop.integral(), 0.0f);
}

TEST(SlewTowardTest, LimitsStepBothWays) {
  EXPECT_FLOAT_EQ(slewToward(0.0f, 10.0f, 1.0f), 1.0f);
  EXPECT_FLOAT_EQ(slewToward(5.0f, -10.0f, 2.0f), 3.0f);
  EXPECT_FLOAT_EQ(slewToward(5.0f, 5.5f, 2.0f), 5.5f);
  EXPECT_FLOAT_EQ(slewToward(5.0f, -10.0f, 0.0f), 5.0f);  // no time elapsed: no step
  EXPECT_FLOAT_EQ(slewToward(5.0f, -10.0f, -1.0f), 5.0f);
}

}  // namespace
}  // namespace cb
