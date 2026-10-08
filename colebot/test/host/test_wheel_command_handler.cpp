// SPDX-License-Identifier: GPL-3.0-or-later
#include <cmath>
#include <limits>
#include <string_view>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "colebot/loop_timing.h"
#include "colebot/wheel_command_handler.h"
#include "mock_drive_controller.h"

namespace cb {
namespace {

using ::testing::_;
using ::testing::FloatEq;
using ::testing::NiceMock;
using ::testing::Return;
using std::string_view_literals::operator""sv;

class WheelCommandHandlerTest : public ::testing::Test {
 protected:
  WheelCommandHandlerTest() {
    ON_CALL(drive_, setWheelVelocities(_, _)).WillByDefault(Return(CommandResult::kAccepted));
  }

  NiceMock<MockDriveController> drive_;
  WheelCommandHandler handler_{drive_, "left_wheel_joint", "right_wheel_joint"};
};

TEST_F(WheelCommandHandlerTest, ForwardsByNameInEitherOrder) {
  const std::string_view names[] = {"right_wheel_joint"sv, "left_wheel_joint"sv};
  const double velocities[] = {-2.0, 3.0};
  EXPECT_CALL(drive_, setWheelVelocities(FloatEq(3.0f), FloatEq(-2.0f)));
  EXPECT_EQ(handler_.handle(names, 2, velocities, 2), CommandResult::kAccepted);
  EXPECT_EQ(handler_.rejectedCount(), 0u);
}

TEST_F(WheelCommandHandlerTest, PassesDriveResultThrough) {
  const std::string_view names[] = {"left_wheel_joint"sv, "right_wheel_joint"sv};
  const double velocities[] = {1.0, 1.0};
  EXPECT_CALL(drive_, setWheelVelocities(_, _)).WillOnce(Return(CommandResult::kEstopLatched));
  EXPECT_EQ(handler_.handle(names, 2, velocities, 2), CommandResult::kEstopLatched);
}

TEST_F(WheelCommandHandlerTest, RejectsMalformedMessages) {
  EXPECT_CALL(drive_, setWheelVelocities(_, _)).Times(0);
  const double velocities[] = {1.0, 1.0, 1.0};

  const std::string_view one[] = {"left_wheel_joint"sv};
  EXPECT_EQ(handler_.handle(one, 1, velocities, 1), CommandResult::kInvalid);  // missing joint

  const std::string_view unknown[] = {"left_wheel_joint"sv, "arm_joint"sv};
  EXPECT_EQ(handler_.handle(unknown, 2, velocities, 2), CommandResult::kInvalid);

  const std::string_view duplicate[] = {"left_wheel_joint"sv, "left_wheel_joint"sv};
  EXPECT_EQ(handler_.handle(duplicate, 2, velocities, 2), CommandResult::kInvalid);

  const std::string_view both[] = {"left_wheel_joint"sv, "right_wheel_joint"sv};
  EXPECT_EQ(handler_.handle(both, 2, velocities, 0), CommandResult::kInvalid);  // no velocities

  const std::string_view three[] = {"left_wheel_joint"sv, "right_wheel_joint"sv, "x"sv};
  EXPECT_EQ(handler_.handle(three, 3, velocities, 3), CommandResult::kInvalid);

  const double nan[] = {std::numeric_limits<double>::quiet_NaN(), 0.0};
  EXPECT_EQ(handler_.handle(both, 2, nan, 2), CommandResult::kInvalid);
  const double inf[] = {0.0, -std::numeric_limits<double>::infinity()};
  EXPECT_EQ(handler_.handle(both, 2, inf, 2), CommandResult::kInvalid);

  EXPECT_EQ(handler_.rejectedCount(), 7u);
}

TEST(StatePublishPolicyTest, PublishesOnEveryCommand) {
  StatePublishPolicy policy(10);
  EXPECT_TRUE(policy.onCommand(0));
  EXPECT_TRUE(policy.onCommand(20000));
}

TEST(StatePublishPolicyTest, IdleTimerOnlyWhenCommandsStop) {
  StatePublishPolicy policy(10);  // 100 ms
  EXPECT_TRUE(policy.onTimer(0));  // nothing published yet
  EXPECT_FALSE(policy.onTimer(50000));
  EXPECT_TRUE(policy.onTimer(100000));
  // Commands at 50 Hz keep the idle timer quiet.
  for (int64_t t = 120000; t < 400000; t += 20000) {
    policy.onCommand(t);
    EXPECT_FALSE(policy.onTimer(t + 10000));
  }
  EXPECT_TRUE(policy.onTimer(500000));
}

TEST(StatePublishPolicyTest, ZeroIdleRateDisablesTimer) {
  StatePublishPolicy policy(0);
  EXPECT_FALSE(policy.onTimer(0));
  EXPECT_FALSE(policy.onTimer(10000000));
}

TEST(LoopTimerTest, TracksMaxPeriodAndMissedDeadlines) {
  LoopTimer timer(1000);
  int64_t t = 0;
  timer.record(t);
  for (int i = 0; i < 10; ++i) timer.record(t += 1000);
  EXPECT_EQ(timer.maxPeriodUs(), 1000);
  EXPECT_EQ(timer.missedDeadlines(), 0u);
  timer.record(t += 1400);  // late but within 1.5x
  timer.record(t += 2600);  // missed
  EXPECT_EQ(timer.maxPeriodUs(), 2600);
  EXPECT_EQ(timer.missedDeadlines(), 1u);
  timer.resetMax();
  timer.record(t += 1000);
  EXPECT_EQ(timer.maxPeriodUs(), 1000);
}

}  // namespace
}  // namespace cb
