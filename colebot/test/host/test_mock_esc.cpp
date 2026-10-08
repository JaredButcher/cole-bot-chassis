// SPDX-License-Identifier: GPL-3.0-or-later
//
// Smoke test for the test doubles (docs/plans/alfredo-dshot.md task A3): the mocks
// compile against the interfaces and can be driven through IEsc / IClock.
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "mock_clock.h"
#include "mock_esc.h"

namespace colebot {
namespace {

using ::testing::Return;

TEST(MockEscTest, DrivesThroughInterface) {
  MockEsc mock;
  EXPECT_CALL(mock, send(1048)).WillOnce(Return(true));
  EXPECT_CALL(mock, isArmed()).WillOnce(Return(false));

  IEsc& esc = mock;
  EXPECT_TRUE(esc.send(1048));
  EXPECT_FALSE(esc.isArmed());
}

TEST(MockClockTest, ReturnsCannedTime) {
  MockClock mock;
  ON_CALL(mock, nowUs()).WillByDefault(Return(1234));

  const IClock& clock = mock;
  EXPECT_EQ(clock.nowUs(), 1234);
}

}  // namespace
}  // namespace colebot
