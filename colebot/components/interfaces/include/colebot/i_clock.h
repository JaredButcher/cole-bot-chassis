// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>

namespace colebot {

// Monotonic time source. Safe to call from any task.
class IClock {
 public:
  virtual ~IClock() = default;
  virtual int64_t nowUs() const = 0;
};

}  // namespace colebot
