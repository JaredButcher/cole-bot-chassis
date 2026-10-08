// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>

#include "colebot/drive_types.h"

namespace cb {

// One wheel: velocity loop, DShot output and telemetry feedback.
// update() and beginEscRestart() run on the motor task only; setParams() and
// escReady() may be called from any task.
class IMotor {
 public:
  virtual ~IMotor() = default;
  // Runs one tick: velocity loop toward velocity_cmd (rad/s, wheel frame), sends
  // one DShot frame and reads the reply. A command below min_speed sends stop.
  virtual WheelSample update(float velocity_cmd, int64_t now_us) = 0;
  // Starts the ESC restart after its power returns: pulls the line low now;
  // update() calls begin() 2.5 s later and the restart ends when the ESC arms.
  virtual void beginEscRestart(int64_t now_us) = 0;
  virtual bool escReady() const = 0;
  virtual void setParams(const MotorParams& params) = 0;
};

}  // namespace cb
