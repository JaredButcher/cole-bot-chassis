// SPDX-License-Identifier: GPL-3.0-or-later
#include "colebot/feedback_store.h"

namespace cb {

void FeedbackStore::push(const WheelSample& left, const WheelSample& right) {
  std::lock_guard<std::mutex> lock(mutex_);
  samples_[index(Side::kLeft)] = left;
  samples_[index(Side::kRight)] = right;
}

WheelSample FeedbackStore::latest(Side side) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_[index(side)];
}

}  // namespace cb
