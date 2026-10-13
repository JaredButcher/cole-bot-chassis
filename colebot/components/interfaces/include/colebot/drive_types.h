// SPDX-License-Identifier: GPL-3.0-or-later
//
// Value types shared by the drive interfaces. Plain data, no behavior.
#pragma once

#include <cstdint>

#include "colebot/i_esc.h"

namespace cb {

enum class Side : uint8_t { kLeft = 0, kRight = 1 };
inline constexpr int kSideCount = 2;
inline constexpr int index(Side side) { return static_cast<int>(side); }

// One wheel's state after a motor tick. Velocities are in the wheel frame
// (positive = robot forward), after inversion.
struct WheelSample {
  int64_t time_us = 0;
  float velocity = 0.0f;      // rad/s, low-pass filtered (what the velocity loop uses)
  float raw_velocity = 0.0f;  // rad/s, latest measurement, held through short dropouts
  double position = 0.0;      // rad, integrated on the MCU since boot
  float throttle = 0.0f;      // last throttle sent, -1..1 (wheel frame)
  bool valid = false;         // telemetry fresh; false once the hold time has run out
  bool esc_ready = false;     // ESC armed and not restarting
  EscRxStatus status = EscRxStatus::kIdle;
  EscLinkStats link{};
};

// Runtime parameters for one wheel's motor and velocity loop (PLAN.md §4.2, §7.2).
struct MotorParams {
  uint8_t motor_poles = 14;
  float drive_reduction = 1.0f;       // motor revs per wheel rev
  bool invert = false;                // firmware fallback; prefer the ESC spin direction
  float kv = 0.0f;                    // feedforward throttle per rad/s
  float ks = 0.0f;                    // static-friction throttle, applied with the command's sign
  float kp = 0.0f;                    // PI gains; 0 = pure feedforward
  float ki = 0.0f;
  float min_speed = 0.0f;             // rad/s; smaller commands count as zero
  float velocity_filter_tau_s = 0.01f;
  uint32_t telemetry_hold_us = 20000;  // keep the last valid speed this long through dropouts
  float direction_flip_speed = 2.0f;  // rad/s; reported direction may only flip below this
};

}  // namespace cb
