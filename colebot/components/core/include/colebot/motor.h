// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>

#include "colebot/drive_math.h"
#include "colebot/i_esc.h"
#include "colebot/i_motor.h"

namespace cb {

// IMotor over one IEsc (PLAN.md §7.1, §7.2, §7.4). The ESC must already have
// been started (releaseBootloader + begin) before the first update().
class Motor final : public IMotor {
 public:
  static constexpr int64_t kRestartHoldUs = 2500000;  // AM32 bootloader release window

  Motor(IEsc& esc, const EscConfig& esc_config, const MotorParams& params);

  WheelSample update(float velocity_cmd, int64_t now_us) override;
  void beginEscRestart(int64_t now_us) override;
  bool escReady() const override;
  void setParams(const MotorParams& params) override;

 private:
  enum class Restart : uint8_t { kNone, kHoldingLow, kWaitingForArm };

  MotorParams paramsSnapshot() const;
  bool computeReady() const;
  float runVelocityLoop(float velocity_cmd, float dt_s, const MotorParams& params);
  void measure(int esc_direction_cmd, int64_t now_us, float dt_s, const MotorParams& params);

  IEsc& esc_;
  const EscConfig esc_config_;

  mutable std::mutex params_mutex_;
  MotorParams params_;

  PiController loop_;
  DirectionTracker direction_;
  Restart restart_ = Restart::kNone;
  std::atomic<bool> ready_{false};  // escReady() for other tasks
  int64_t restart_start_us_ = 0;
  int64_t last_update_us_ = -1;
  int64_t last_valid_us_ = -1;
  float speed_ = 0.0f;  // measured magnitude, held through short dropouts
  WheelSample sample_;
};

}  // namespace cb
