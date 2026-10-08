// SPDX-License-Identifier: GPL-3.0-or-later
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "colebot/motor.h"
#include "mock_esc.h"

namespace cb {
namespace {

using ::testing::_;
using ::testing::InSequence;
using ::testing::NiceMock;
using ::testing::Return;

constexpr int64_t kTickUs = 1000;
constexpr EscConfig kEscConfig{DshotMode::k600, 14, false};

// 7000 eRPM with 14 magnets at 1:1 is 104.72 rad/s.
constexpr uint32_t kErpm100 = 6685;  // ~100 rad/s

class MotorTest : public ::testing::Test {
 protected:
  MotorTest() {
    params_.kv = 0.01f;  // 50 rad/s -> 0.5 throttle
    ON_CALL(esc_, isArmed()).WillByDefault(Return(true));
    setTelemetry(EscRxStatus::kOk, 0);
  }

  void setTelemetry(EscRxStatus status, uint32_t erpm) {
    ON_CALL(esc_, telemetry()).WillByDefault(Return(EscTelemetry{status, erpm, 0}));
  }

  WheelSample tick(Motor& motor, float cmd) {
    now_ += kTickUs;
    return motor.update(cmd, now_);
  }

  NiceMock<MockEsc> esc_;
  MotorParams params_;
  int64_t now_ = 0;
};

TEST_F(MotorTest, VelocityCommandMapsTo3dDshot) {
  Motor motor(esc_, kEscConfig, params_);
  EXPECT_CALL(esc_, send(1548));  // +0.5
  EXPECT_CALL(esc_, send(548));   // -0.5
  EXPECT_CALL(esc_, send(0));
  tick(motor, 50.0f);
  tick(motor, -50.0f);
  tick(motor, 0.0f);
}

TEST_F(MotorTest, BelowMinSpeedSendsStop) {
  params_.min_speed = 3.0f;
  Motor motor(esc_, kEscConfig, params_);
  EXPECT_CALL(esc_, send(0));
  EXPECT_EQ(tick(motor, 2.0f).throttle, 0.0f);
}

TEST_F(MotorTest, NotArmedSendsStop) {
  ON_CALL(esc_, isArmed()).WillByDefault(Return(false));
  Motor motor(esc_, kEscConfig, params_);
  EXPECT_CALL(esc_, send(0));
  const WheelSample sample = tick(motor, 50.0f);
  EXPECT_FALSE(sample.esc_ready);
  EXPECT_FALSE(motor.escReady());
}

TEST_F(MotorTest, InversionFlipsThrottleAndMeasuredSign) {
  params_.invert = true;
  Motor motor(esc_, kEscConfig, params_);
  EXPECT_CALL(esc_, send(548)).Times(2);  // ESC runs reverse for wheel-forward
  tick(motor, 50.0f);                     // from rest
  setTelemetry(EscRxStatus::kOk, kErpm100);
  const WheelSample sample = tick(motor, 50.0f);
  EXPECT_GT(sample.raw_velocity, 99.0f);  // reported in the wheel frame: forward
  EXPECT_FLOAT_EQ(sample.throttle, 0.5f);
}

TEST_F(MotorTest, MeasuresWheelSpeedFromErpm) {
  params_.drive_reduction = 2.0f;
  Motor motor(esc_, kEscConfig, params_);
  setTelemetry(EscRxStatus::kOk, 7000);
  const WheelSample sample = tick(motor, 50.0f);
  EXPECT_NEAR(sample.raw_velocity, 104.7198f / 2.0f, 1e-3f);
  EXPECT_TRUE(sample.valid);
}

TEST_F(MotorTest, DirectionHoldsThroughReversalUntilSlow) {
  Motor motor(esc_, kEscConfig, params_);
  setTelemetry(EscRxStatus::kOk, kErpm100);
  tick(motor, 50.0f);
  // Command reverse while the wheel still spins forward: speed stays positive.
  EXPECT_GT(tick(motor, -50.0f).raw_velocity, 0.0f);
  setTelemetry(EscRxStatus::kOk, 100);  // ~1.5 rad/s, below the flip speed
  EXPECT_LT(tick(motor, -50.0f).raw_velocity, 0.0f);
  setTelemetry(EscRxStatus::kOk, kErpm100);
  EXPECT_LT(tick(motor, -50.0f).raw_velocity, -99.0f);
}

TEST_F(MotorTest, IntegratesPositionAndFiltersVelocity) {
  Motor motor(esc_, kEscConfig, params_);
  setTelemetry(EscRxStatus::kOk, kErpm100);
  const float speed = erpmToWheelSpeed(kErpm100, 14, 1.0f);
  WheelSample sample;
  for (int i = 0; i <= 1000; ++i) sample = tick(motor, 50.0f);  // 1 s after the first tick
  EXPECT_NEAR(sample.position, speed * 1.0, 0.01);
  EXPECT_NEAR(sample.velocity, speed, 0.01f);  // filter has settled (tau 10 ms)
}

TEST_F(MotorTest, PositionCountsBackwardThroughReversal) {
  Motor motor(esc_, kEscConfig, params_);
  setTelemetry(EscRxStatus::kOk, kErpm100);
  for (int i = 0; i <= 500; ++i) tick(motor, 50.0f);
  setTelemetry(EscRxStatus::kOk, 0);
  tick(motor, -50.0f);  // stopped: direction flips
  setTelemetry(EscRxStatus::kOk, kErpm100);
  WheelSample sample;
  for (int i = 0; i < 500; ++i) sample = tick(motor, -50.0f);
  EXPECT_NEAR(sample.position, 0.0, 0.25);  // forward 0.5 s, back 0.5 s
}

TEST_F(MotorTest, HoldsSpeedThroughShortDropoutThenInvalid) {
  Motor motor(esc_, kEscConfig, params_);
  setTelemetry(EscRxStatus::kOk, kErpm100);
  tick(motor, 50.0f);
  setTelemetry(EscRxStatus::kNoReply, 0);
  WheelSample sample = tick(motor, 50.0f);
  EXPECT_TRUE(sample.valid);
  EXPECT_GT(sample.raw_velocity, 99.0f);
  EXPECT_EQ(sample.status, EscRxStatus::kNoReply);
  for (int i = 0; i < 25; ++i) sample = tick(motor, 50.0f);  // past the 20 ms hold
  EXPECT_FALSE(sample.valid);
  EXPECT_EQ(sample.raw_velocity, 0.0f);
}

TEST_F(MotorTest, EscRestartHoldsLowThenBeginsAndWaitsForArm) {
  Motor motor(esc_, kEscConfig, params_);
  tick(motor, 0.0f);

  bool armed = false;
  ON_CALL(esc_, isArmed()).WillByDefault([&] { return armed; });
  {
    InSequence order;
    EXPECT_CALL(esc_, holdLineLow());
    EXPECT_CALL(esc_, send(_)).Times(0);
  }
  motor.beginEscRestart(now_);
  EXPECT_FALSE(motor.escReady());
  for (int i = 0; i < 2499; ++i) EXPECT_FALSE(tick(motor, 50.0f).esc_ready);
  ::testing::Mock::VerifyAndClearExpectations(&esc_);

  // Hold over: begin() with the original config, then zero throttle until armed.
  EXPECT_CALL(esc_, begin(::testing::Field(&EscConfig::motor_poles, 14))).WillOnce(Return(true));
  EXPECT_CALL(esc_, send(0)).Times(2);
  tick(motor, 50.0f);
  tick(motor, 50.0f);
  ::testing::Mock::VerifyAndClearExpectations(&esc_);

  armed = true;
  EXPECT_CALL(esc_, send(1548));
  EXPECT_TRUE(tick(motor, 50.0f).esc_ready);
  EXPECT_TRUE(motor.escReady());
}

TEST_F(MotorTest, ParamsApplyOnNextTick) {
  Motor motor(esc_, kEscConfig, params_);
  EXPECT_CALL(esc_, send(1548));
  tick(motor, 50.0f);
  params_.kv = 0.02f;
  motor.setParams(params_);
  EXPECT_CALL(esc_, send(2047));
  tick(motor, 50.0f);
}

}  // namespace
}  // namespace cb
