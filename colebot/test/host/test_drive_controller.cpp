// SPDX-License-Identifier: GPL-3.0-or-later
#include <cmath>
#include <limits>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "colebot/drive_controller.h"
#include "colebot/feedback_store.h"
#include "fake_clock.h"
#include "fake_estop_input.h"
#include "mock_motor.h"

namespace cb {
namespace {

using ::testing::_;
using ::testing::FloatEq;
using ::testing::NiceMock;
using ::testing::Return;

class DriveControllerTest : public ::testing::Test {
 protected:
  DriveControllerTest() {
    for (auto* motor : {&left_, &right_}) {
      ON_CALL(*motor, escReady()).WillByDefault([this, motor] {
        return motor == &left_ ? left_ready_ : right_ready_;
      });
      ON_CALL(*motor, beginEscRestart(_)).WillByDefault([this, motor](int64_t) {
        (motor == &left_ ? left_ready_ : right_ready_) = false;
      });
    }
    params_.max_wheel_accel = 0.0f;  // no ramp unless a test sets one
  }

  DriveController make() { return DriveController(left_, right_, clock_, estop_input_, params_); }

  // Runs ticks at 1 kHz and returns the last setpoints the motors received.
  void run(DriveController& drive, int ticks) {
    for (int i = 0; i < ticks; ++i) {
      clock_.advanceUs(1000);
      drive.tick();
    }
  }

  float setpoint(const DriveController& drive, Side side) {
    return drive.state().setpoint[index(side)];
  }

  NiceMock<MockMotor> left_;
  NiceMock<MockMotor> right_;
  bool left_ready_ = true;
  bool right_ready_ = true;
  FakeClock clock_;
  FakeEstopInput estop_input_;
  DriveParams params_;
};

TEST_F(DriveControllerTest, PassesSetpointsToMotors) {
  DriveController drive = make();
  EXPECT_EQ(drive.setWheelVelocities(5.0f, -3.0f), CommandResult::kAccepted);
  EXPECT_CALL(left_, update(FloatEq(5.0f), _));
  EXPECT_CALL(right_, update(FloatEq(-3.0f), _));
  run(drive, 1);
}

TEST_F(DriveControllerTest, ClampsToMaxSpeedAndRejectsNonFinite) {
  params_.max_wheel_speed = 10.0f;
  DriveController drive = make();
  drive.setWheelVelocities(50.0f, -50.0f);
  run(drive, 1);
  EXPECT_FLOAT_EQ(setpoint(drive, Side::kLeft), 10.0f);
  EXPECT_FLOAT_EQ(setpoint(drive, Side::kRight), -10.0f);
  EXPECT_EQ(drive.setWheelVelocities(std::numeric_limits<float>::quiet_NaN(), 0.0f),
            CommandResult::kInvalid);
  EXPECT_EQ(drive.setWheelVelocities(0.0f, std::numeric_limits<float>::infinity()),
            CommandResult::kInvalid);
}

TEST_F(DriveControllerTest, RampLimitsAcceleration) {
  params_.max_wheel_accel = 100.0f;  // 0.1 rad/s per 1 ms tick
  DriveController drive = make();
  drive.setWheelVelocities(10.0f, -10.0f);
  run(drive, 11);  // the first tick has no elapsed time, so it doesn't step
  EXPECT_NEAR(setpoint(drive, Side::kLeft), 1.0f, 1e-4f);
  EXPECT_NEAR(setpoint(drive, Side::kRight), -1.0f, 1e-4f);
}

TEST_F(DriveControllerTest, TimeoutRampsToZeroAndOnlyCommandsRenew) {
  params_.cmd_timeout_ms = 250;
  DriveController drive = make();
  drive.setWheelVelocities(5.0f, 5.0f);
  run(drive, 200);
  EXPECT_FLOAT_EQ(setpoint(drive, Side::kLeft), 5.0f);
  drive.state();  // reads don't renew
  drive.isStopped();
  run(drive, 60);
  EXPECT_FLOAT_EQ(setpoint(drive, Side::kLeft), 0.0f);
  EXPECT_FALSE(drive.state().commanded);
  EXPECT_EQ(drive.state().timeout_count, 1u);
  EXPECT_FALSE(drive.state().estop_latched);  // a timeout is not an e-stop

  // A new command drives again.
  drive.setWheelVelocities(5.0f, 5.0f);
  run(drive, 1);
  EXPECT_FLOAT_EQ(setpoint(drive, Side::kLeft), 5.0f);
}

TEST_F(DriveControllerTest, RenewedCommandsKeepDriving) {
  DriveController drive = make();
  for (int i = 0; i < 50; ++i) {
    drive.setWheelVelocities(5.0f, 5.0f);
    run(drive, 20);  // 50 Hz
  }
  EXPECT_FLOAT_EQ(setpoint(drive, Side::kLeft), 5.0f);
  EXPECT_EQ(drive.state().timeout_count, 0u);
}

TEST_F(DriveControllerTest, SessionLossStopsWithoutWaitingForTimeout) {
  params_.max_wheel_accel = 100.0f;
  DriveController drive = make();
  drive.setWheelVelocities(1.0f, 1.0f);
  run(drive, 20);
  drive.stopForSessionLoss();
  run(drive, 20);  // well inside the 250 ms timeout
  EXPECT_FLOAT_EQ(setpoint(drive, Side::kLeft), 0.0f);
}

TEST_F(DriveControllerTest, EstopZeroesImmediatelyAndRecordsSources) {
  params_.max_wheel_accel = 1.0f;  // a ramp that would take seconds
  DriveController drive = make();
  for (int i = 0; i < 50; ++i) {
    drive.setWheelVelocities(0.5f, 0.5f);
    run(drive, 20);
  }
  ASSERT_GT(setpoint(drive, Side::kLeft), 0.4f);

  drive.estop(StopSource::kConsole);
  drive.estop(StopSource::kRos);
  run(drive, 1);
  const DriveState state = drive.state();
  EXPECT_FLOAT_EQ(state.setpoint[0], 0.0f);
  EXPECT_TRUE(state.estop_latched);
  EXPECT_EQ(state.first_source, StopSource::kConsole);
  EXPECT_EQ(state.source_mask,
            static_cast<uint8_t>(StopSource::kConsole) | static_cast<uint8_t>(StopSource::kRos));
  EXPECT_EQ(drive.setWheelVelocities(1.0f, 1.0f), CommandResult::kEstopLatched);
}

TEST_F(DriveControllerTest, ClearRequiresLatchAndHoldsUntilZeroCommand) {
  DriveController drive = make();
  EXPECT_EQ(drive.clearEstop(), ClearResult::kNotLatched);

  drive.setWheelVelocities(5.0f, 5.0f);
  drive.estop(StopSource::kRos);
  EXPECT_EQ(drive.clearEstop(), ClearResult::kCleared);
  EXPECT_FALSE(drive.state().estop_latched);
  EXPECT_EQ(drive.state().first_source, StopSource::kNone);

  // The host is still streaming its old command: the drive stays stopped.
  EXPECT_EQ(drive.setWheelVelocities(5.0f, 5.0f), CommandResult::kHeldUntilZero);
  run(drive, 10);
  EXPECT_FLOAT_EQ(setpoint(drive, Side::kLeft), 0.0f);
  EXPECT_TRUE(drive.state().held_until_zero);

  EXPECT_EQ(drive.setWheelVelocities(0.0f, 0.0f), CommandResult::kAccepted);
  EXPECT_EQ(drive.setWheelVelocities(5.0f, 5.0f), CommandResult::kAccepted);
  run(drive, 1);
  EXPECT_FLOAT_EQ(setpoint(drive, Side::kLeft), 5.0f);
}

TEST_F(DriveControllerTest, AutomaticTripLatchesAndBlocksClear) {
  DriveController drive = make();
  drive.setTrip(StopSource::kLowBattery, true);
  EXPECT_TRUE(drive.state().estop_latched);
  EXPECT_EQ(drive.state().first_source, StopSource::kLowBattery);
  EXPECT_EQ(drive.clearEstop(), ClearResult::kTripActive);
  drive.setTrip(StopSource::kLowBattery, false);
  EXPECT_EQ(drive.clearEstop(), ClearResult::kCleared);
}

TEST_F(DriveControllerTest, HardwareEstopLatchesOnFirstPressedSample) {
  DriveController drive = make();
  drive.setWheelVelocities(5.0f, 5.0f);
  run(drive, 5);
  estop_input_.press();
  run(drive, 1);
  const DriveState state = drive.state();
  EXPECT_TRUE(state.estop_latched);
  EXPECT_EQ(state.first_source, StopSource::kHardware);
  EXPECT_EQ(state.active_trips, static_cast<uint8_t>(StopSource::kHardware));
  EXPECT_FLOAT_EQ(state.setpoint[0], 0.0f);
  EXPECT_EQ(drive.clearEstop(), ClearResult::kTripActive);
}

TEST_F(DriveControllerTest, HardwareReleaseIsDebouncedAndRestartsEscs) {
  DriveController drive = make();
  estop_input_.press();
  run(drive, 10);

  // A bounce shorter than the debounce time doesn't end the trip.
  estop_input_.release();
  run(drive, 10);
  estop_input_.press();
  run(drive, 1);
  EXPECT_TRUE(drive.state().active_trips & static_cast<uint8_t>(StopSource::kHardware));

  estop_input_.release();
  EXPECT_CALL(left_, beginEscRestart(_)).Times(1);
  EXPECT_CALL(right_, beginEscRestart(_)).Times(1);
  run(drive, 25);
  DriveState state = drive.state();
  EXPECT_EQ(state.active_trips, static_cast<uint8_t>(StopSource::kEscRestart));
  EXPECT_TRUE(state.estop_latched);  // release alone doesn't clear the latch
  EXPECT_EQ(drive.clearEstop(), ClearResult::kTripActive);

  // ESCs armed again: the restart trip ends and a clear is accepted.
  left_ready_ = right_ready_ = true;
  run(drive, 1);
  EXPECT_EQ(drive.state().active_trips, 0u);
  EXPECT_EQ(drive.clearEstop(), ClearResult::kCleared);
}

TEST_F(DriveControllerTest, PressDuringEscRestartRestartsAgainOnRelease) {
  DriveController drive = make();
  estop_input_.press();
  run(drive, 1);
  estop_input_.release();
  EXPECT_CALL(left_, beginEscRestart(_)).Times(2);
  EXPECT_CALL(right_, beginEscRestart(_)).Times(2);
  run(drive, 25);
  estop_input_.press();
  run(drive, 5);
  left_ready_ = right_ready_ = true;  // the old restart finishing must not end the trip
  run(drive, 5);
  EXPECT_NE(drive.state().active_trips & static_cast<uint8_t>(StopSource::kEscRestart), 0);
  estop_input_.release();
  run(drive, 25);
}

TEST_F(DriveControllerTest, IsStopped) {
  params_.max_wheel_accel = 100.0f;
  DriveController drive = make();
  EXPECT_TRUE(drive.isStopped());
  drive.setWheelVelocities(1.0f, 0.0f);
  EXPECT_FALSE(drive.isStopped());  // about to move
  run(drive, 100);
  drive.setWheelVelocities(0.0f, 0.0f);
  run(drive, 1);
  EXPECT_FALSE(drive.isStopped());  // still ramping down
  run(drive, 20);
  EXPECT_TRUE(drive.isStopped());
}

TEST(FeedbackStoreTest, StoresLatestPerSide) {
  FeedbackStore store;
  WheelSample left;
  left.position = 1.5;
  WheelSample right;
  right.position = -2.5;
  store.push(left, right);
  EXPECT_DOUBLE_EQ(store.latest(Side::kLeft).position, 1.5);
  EXPECT_DOUBLE_EQ(store.latest(Side::kRight).position, -2.5);
}

}  // namespace
}  // namespace cb
