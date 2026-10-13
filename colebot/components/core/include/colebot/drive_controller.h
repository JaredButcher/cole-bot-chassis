// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <mutex>

#include "colebot/drive_types.h"
#include "colebot/i_clock.h"
#include "colebot/i_drive_controller.h"
#include "colebot/i_estop_input.h"
#include "colebot/i_motor.h"

namespace cb {

struct DriveTick {
  WheelSample left;
  WheelSample right;
};

// IDriveController (PLAN.md §7.3): command timeout, e-stop latch, hardware e-stop
// input and safety ramp in front of two motors. tick() runs on the motor task.
class DriveController final : public IDriveController {
 public:
  static constexpr int64_t kEstopReleaseDebounceUs = 20000;
  static constexpr float kZeroCommand = 1e-3f;  // rad/s; "zero" for the rule after a clear

  DriveController(IMotor& left, IMotor& right, IClock& clock, IEstopInput& estop_input,
                  const DriveParams& params);

  // One motor-task tick: samples the e-stop input, decides the setpoints and
  // runs both motors.
  DriveTick tick();

  CommandResult setWheelVelocities(float left, float right) override;
  void stopForSessionLoss() override;
  void estop(StopSource source) override;
  ClearResult clearEstop() override;
  void setTrip(StopSource trip, bool active) override;
  void setParams(const DriveParams& params) override;
  DriveState state() const override;
  bool isStopped() const override;

 private:
  void latchLocked(StopSource source);
  bool hasTripLocked(StopSource trip) const;
  void setTripBitLocked(StopSource trip, bool active);

  IMotor* motors_[kSideCount];
  IClock& clock_;
  IEstopInput& estop_input_;

  mutable std::mutex mutex_;
  DriveParams params_;
  DriveState state_;
  float command_[kSideCount] = {0.0f, 0.0f};
  int64_t command_deadline_us_ = 0;
  int64_t last_tick_us_ = -1;
  int64_t estop_released_since_us_ = -1;
};

}  // namespace cb
