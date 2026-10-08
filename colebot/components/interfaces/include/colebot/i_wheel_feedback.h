// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "colebot/drive_types.h"

namespace cb {

// Latest wheel samples, written by the motor task and read by the ros and console
// tasks. Safe to call from any task.
class IWheelFeedback {
 public:
  virtual ~IWheelFeedback() = default;
  virtual WheelSample latest(Side side) const = 0;
};

}  // namespace cb
