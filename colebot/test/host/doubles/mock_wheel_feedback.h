// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <gmock/gmock.h>

#include "colebot/i_wheel_feedback.h"

namespace cb {

class MockWheelFeedback : public IWheelFeedback {
 public:
  MOCK_METHOD(WheelSample, latest, (Side side), (const, override));
};

}  // namespace cb
