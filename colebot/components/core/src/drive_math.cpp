// SPDX-License-Identifier: GPL-3.0-or-later
#include "colebot/drive_math.h"

#include <algorithm>
#include <cmath>

namespace cb {

namespace {
constexpr float kTwoPi = 6.28318530718f;

float signOf(float value) { return value > 0.0f ? 1.0f : (value < 0.0f ? -1.0f : 0.0f); }
}  // namespace

float erpmToWheelSpeed(uint32_t erpm, uint8_t motor_poles, float drive_reduction) {
  if (motor_poles == 0 || !(drive_reduction > 0.0f)) return 0.0f;
  const float motor_rpm = static_cast<float>(erpm) * 2.0f / motor_poles;
  return motor_rpm / drive_reduction * kTwoPi / 60.0f;
}

int DirectionTracker::update(int commanded, float speed, float flip_speed) {
  if (commanded != 0 && commanded != direction_ && speed < flip_speed) {
    direction_ = commanded > 0 ? 1 : -1;
  }
  return direction_;
}

float PiController::update(float target, float measured, float dt_s,
                           const VelocityLoopGains& gains) {
  if (!(std::fabs(target) >= gains.min_speed) || target == 0.0f) {  // also catches NaN
    reset();
    return 0.0f;
  }
  const float feedforward = gains.kv * target + gains.ks * signOf(target);
  const float error = target - measured;
  const float proportional = gains.kp * error;

  const float candidate = integral_ + error * std::max(dt_s, 0.0f);
  const float unclamped = feedforward + proportional + gains.ki * candidate;
  const bool saturated = std::fabs(unclamped) > 1.0f;
  const bool winding_up = signOf(error) == signOf(unclamped);
  if (!saturated || !winding_up) integral_ = candidate;

  const float output = feedforward + proportional + gains.ki * integral_;
  return std::clamp(output, -1.0f, 1.0f);
}

float slewToward(float current, float target, float max_step) {
  const float step = max_step > 0.0f ? max_step : 0.0f;  // also catches NaN
  return current + std::clamp(target - current, -step, step);
}

}  // namespace cb
