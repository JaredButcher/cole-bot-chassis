// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <gmock/gmock.h>

#include "colebot/i_drive_controller.h"

namespace cb {

class MockDriveController : public IDriveController {
 public:
  MOCK_METHOD(CommandResult, setWheelVelocities, (float left, float right), (override));
  MOCK_METHOD(void, stopForSessionLoss, (), (override));
  MOCK_METHOD(void, estop, (StopSource source), (override));
  MOCK_METHOD(ClearResult, clearEstop, (), (override));
  MOCK_METHOD(void, setTrip, (StopSource trip, bool active), (override));
  MOCK_METHOD(void, setParams, (const DriveParams& params), (override));
  MOCK_METHOD(DriveState, state, (), (const, override));
  MOCK_METHOD(bool, isStopped, (), (const, override));
};

}  // namespace cb
