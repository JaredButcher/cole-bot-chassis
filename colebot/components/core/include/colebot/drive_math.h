// SPDX-License-Identifier: GPL-3.0-or-later
//
// Small pure helpers for the drive: unit conversion, reported direction and the
// wheel velocity loop. No state shared between tasks.
#pragma once

#include <cstdint>

namespace cb {

// eRPM from bidirectional DShot telemetry -> wheel angular speed (rad/s, >= 0).
// Returns 0 for a zero magnet count or reduction.
float erpmToWheelSpeed(uint32_t erpm, uint8_t motor_poles, float drive_reduction);

// Telemetry speed is unsigned. When the command changes sign at speed, AM32 first
// slows the motor and only then reverses, so the reported direction follows the
// command only once the measured speed has dropped below flip_speed (PLAN.md §7.4).
class DirectionTracker {
 public:
  // commanded: sign of the throttle being sent (-1, 0 or +1, ESC frame).
  // speed: measured speed magnitude. Returns the current direction (-1 or +1).
  int update(int commanded, float speed, float flip_speed);
  int direction() const { return direction_; }

 private:
  int direction_ = 1;
};

struct VelocityLoopGains {
  float kv = 0.0f;
  float ks = 0.0f;
  float kp = 0.0f;
  float ki = 0.0f;
  float min_speed = 0.0f;
};

// throttle = kv·ω_cmd + ks·sign(ω_cmd) + kp·e + ki·∫e, clamped to ±1, with
// anti-windup: the integrator holds while the output is saturated and the error
// would push it further into saturation (PLAN.md §7.2).
class PiController {
 public:
  float update(float target, float measured, float dt_s, const VelocityLoopGains& gains);
  void reset() { integral_ = 0.0f; }
  float integral() const { return integral_; }

 private:
  float integral_ = 0.0f;
};

// Moves current toward target by at most max_step (a negative step counts as 0).
float slewToward(float current, float target, float max_step);

}  // namespace cb
