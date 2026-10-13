// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>

namespace cb {

// Maps a signed throttle in [-1.0, 1.0] to an AM32 3D-mode DShot value:
//   0            stop (also for 0, NaN)
//   48..1047     reverse, slowest to fastest
//   1048..2047   forward, slowest to fastest
// Values beyond ±1.0 are clamped. Same mapping as throttle3D() in the
// AlfredoDShot Rotini_V4_Telemetry example, which takes percent.
uint16_t throttleTo3dDshot(float throttle);

}  // namespace cb
