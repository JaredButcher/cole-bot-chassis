// SPDX-License-Identifier: GPL-3.0-or-later
#include "colebot/throttle_3d.h"

#include <algorithm>
#include <cmath>

namespace colebot {

namespace {
constexpr uint16_t kReverseMin = 48;
constexpr uint16_t kForwardMin = 1048;
constexpr float kSteps = 999.0f;  // 1000 values per direction
}  // namespace

uint16_t throttleTo3dDshot(float throttle) {
  if (!(std::fabs(throttle) > 0.0f)) return 0;  // also catches NaN
  const float magnitude = std::min(std::fabs(throttle), 1.0f);
  const auto steps = static_cast<uint16_t>(magnitude * kSteps + 0.5f);
  return static_cast<uint16_t>((throttle > 0.0f ? kForwardMin : kReverseMin) + steps);
}

}  // namespace colebot
