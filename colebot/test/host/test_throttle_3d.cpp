// SPDX-License-Identifier: GPL-3.0-or-later
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "colebot/throttle_3d.h"

namespace colebot {
namespace {

TEST(Throttle3dTest, ZeroAndNanStop) {
  EXPECT_EQ(throttleTo3dDshot(0.0f), 0);
  EXPECT_EQ(throttleTo3dDshot(-0.0f), 0);
  EXPECT_EQ(throttleTo3dDshot(std::numeric_limits<float>::quiet_NaN()), 0);
}

TEST(Throttle3dTest, ForwardRange) {
  EXPECT_EQ(throttleTo3dDshot(1e-6f), 1048);  // slowest forward, not stop
  EXPECT_EQ(throttleTo3dDshot(0.5f), 1548);
  EXPECT_EQ(throttleTo3dDshot(1.0f), 2047);
}

TEST(Throttle3dTest, ReverseRange) {
  EXPECT_EQ(throttleTo3dDshot(-1e-6f), 48);  // slowest reverse, not stop
  EXPECT_EQ(throttleTo3dDshot(-0.5f), 548);
  EXPECT_EQ(throttleTo3dDshot(-1.0f), 1047);
}

TEST(Throttle3dTest, ClampsBeyondFullScale) {
  EXPECT_EQ(throttleTo3dDshot(2.0f), 2047);
  EXPECT_EQ(throttleTo3dDshot(-2.0f), 1047);
  EXPECT_EQ(throttleTo3dDshot(std::numeric_limits<float>::infinity()), 2047);
  EXPECT_EQ(throttleTo3dDshot(-std::numeric_limits<float>::infinity()), 1047);
}

// Same values as throttle3D(pct) in AlfredoDShot's Rotini_V4_Telemetry example.
TEST(Throttle3dTest, MatchesLibraryExample) {
  for (int pct = -100; pct <= 100; ++pct) {
    const float p = static_cast<float>(pct);
    uint16_t expected = 0;
    if (pct != 0) {
      const auto steps = static_cast<uint16_t>(std::fabs(p) * 9.99f + 0.5f);
      expected = static_cast<uint16_t>((pct > 0 ? 1048 : 48) + steps);
    }
    EXPECT_EQ(throttleTo3dDshot(p / 100.0f), expected) << "pct=" << pct;
  }
}

}  // namespace
}  // namespace colebot
