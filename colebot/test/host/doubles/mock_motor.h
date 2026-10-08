// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <gmock/gmock.h>

#include "colebot/i_motor.h"

namespace cb {

class MockMotor : public IMotor {
 public:
  MOCK_METHOD(WheelSample, update, (float velocity_cmd, int64_t now_us), (override));
  MOCK_METHOD(void, beginEscRestart, (int64_t now_us), (override));
  MOCK_METHOD(bool, escReady, (), (const, override));
  MOCK_METHOD(void, setParams, (const MotorParams& params), (override));
};

}  // namespace cb
