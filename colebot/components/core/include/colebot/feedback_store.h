// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <mutex>

#include "colebot/i_wheel_feedback.h"

namespace cb {

// Hands the motor task's samples to other tasks (PLAN.md §7.4). Filtering and
// position integration happen in Motor; this only stores the latest snapshot.
class FeedbackStore final : public IWheelFeedback {
 public:
  void push(const WheelSample& left, const WheelSample& right);
  WheelSample latest(Side side) const override;

 private:
  mutable std::mutex mutex_;
  WheelSample samples_[kSideCount];
};

}  // namespace cb
