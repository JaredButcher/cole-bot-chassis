// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "colebot/i_drive_controller.h"

namespace cb {

// Validates a decoded /chassis/wheel_commands message (sensor_msgs/JointState,
// name + velocity) and forwards it to the drive (PLAN.md §4.2). The message must
// name exactly the two wheel joints, in any order, each with a finite velocity.
// RosNode decodes the message; this class holds the rules.
class WheelCommandHandler {
 public:
  WheelCommandHandler(IDriveController& drive, std::string_view left_joint,
                      std::string_view right_joint);

  CommandResult handle(const std::string_view* names, size_t name_count, const double* velocities,
                       size_t velocity_count);

  uint32_t rejectedCount() const { return rejected_.load(); }

 private:
  IDriveController& drive_;
  std::string_view joints_[2];
  std::atomic<uint32_t> rejected_{0};
};

}  // namespace cb
