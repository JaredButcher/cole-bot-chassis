// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "colebot/i_clock.h"

namespace cb {

class FakeClock : public IClock {
 public:
  int64_t nowUs() const override { return now_us_; }
  void advanceUs(int64_t us) { now_us_ += us; }
  void advanceMs(int64_t ms) { now_us_ += ms * 1000; }

 private:
  int64_t now_us_ = 1000000;  // non-zero, like time since boot
};

}  // namespace cb
