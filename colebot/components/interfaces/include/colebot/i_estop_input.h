// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

namespace cb {

// Raw state of the hardware e-stop's auxiliary contact (PLAN.md §7.3). Wired
// fail-safe: an open circuit reads as pressed. Motor task only.
class IEstopInput {
 public:
  virtual ~IEstopInput() = default;
  virtual bool pressed() const = 0;
};

}  // namespace cb
