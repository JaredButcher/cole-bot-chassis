// SPDX-License-Identifier: GPL-3.0-or-later
#include "colebot/wheel_command_handler.h"

#include <cmath>

#include "colebot/drive_types.h"

namespace cb {

WheelCommandHandler::WheelCommandHandler(IDriveController& drive, std::string_view left_joint,
                                         std::string_view right_joint)
    : drive_(drive), joints_{left_joint, right_joint} {}

CommandResult WheelCommandHandler::handle(const std::string_view* names, size_t name_count,
                                          const double* velocities, size_t velocity_count) {
  double velocity[kSideCount] = {0.0, 0.0};
  bool seen[kSideCount] = {false, false};
  bool valid = name_count == kSideCount && velocity_count == kSideCount;
  for (size_t i = 0; valid && i < name_count; ++i) {
    int side = -1;
    for (int s = 0; s < kSideCount; ++s) {
      if (names[i] == joints_[s]) side = s;
    }
    if (side < 0 || seen[side] || !std::isfinite(velocities[i])) {
      valid = false;
    } else {
      seen[side] = true;
      velocity[side] = velocities[i];
    }
  }
  if (!valid) {
    ++rejected_;
    return CommandResult::kInvalid;
  }
  const CommandResult result = drive_.setWheelVelocities(static_cast<float>(velocity[0]),
                                                         static_cast<float>(velocity[1]));
  if (result == CommandResult::kInvalid) ++rejected_;
  return result;
}

}  // namespace cb
