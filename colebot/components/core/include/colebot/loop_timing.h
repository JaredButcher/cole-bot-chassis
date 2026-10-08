// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <atomic>
#include <cstdint>

namespace cb {

// When to publish /chassis/wheel_states (PLAN.md §7.5): right after each accepted
// wheel command, so the rate follows the host's control rate, and from an idle
// timer at idle_hz when no commands arrive. Used by the ros task only.
class StatePublishPolicy {
 public:
  explicit StatePublishPolicy(uint32_t idle_hz) { setIdleHz(idle_hz); }
  void setIdleHz(uint32_t idle_hz) { idle_period_us_ = idle_hz > 0 ? 1000000 / idle_hz : 0; }

  // A command was handled: publish now.
  bool onCommand(int64_t now_us) {
    last_publish_us_ = now_us;
    return true;
  }
  // Periodic check: publish if nothing was published for one idle period.
  bool onTimer(int64_t now_us) {
    if (idle_period_us_ == 0) return false;
    if (last_publish_us_ >= 0 && now_us - last_publish_us_ < idle_period_us_) return false;
    last_publish_us_ = now_us;
    return true;
  }

 private:
  int64_t idle_period_us_ = 0;
  int64_t last_publish_us_ = -1;
};

// Period statistics for a fixed-rate loop (the 1 kHz motor task). record() runs
// on the loop's task; the getters may be read from any task.
class LoopTimer {
 public:
  explicit LoopTimer(int64_t nominal_period_us) : nominal_us_(nominal_period_us) {}

  void record(int64_t now_us) {
    if (last_us_ >= 0) {
      const int64_t period = now_us - last_us_;
      if (period > max_period_us_.load()) max_period_us_ = period;
      if (period > nominal_us_ + nominal_us_ / 2) ++missed_;
    }
    last_us_ = now_us;
  }
  int64_t maxPeriodUs() const { return max_period_us_.load(); }
  uint32_t missedDeadlines() const { return missed_.load(); }
  void resetMax() { max_period_us_ = 0; }

 private:
  const int64_t nominal_us_;
  int64_t last_us_ = -1;
  std::atomic<int64_t> max_period_us_{0};
  std::atomic<uint32_t> missed_{0};
};

}  // namespace cb
