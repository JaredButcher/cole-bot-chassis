// SPDX-License-Identifier: GPL-3.0-or-later
#include "colebot/drive_controller.h"

#include <algorithm>
#include <cmath>
#include <iterator>

#include "colebot/drive_math.h"

namespace cb {

namespace {
uint8_t bit(StopSource source) { return static_cast<uint8_t>(source); }
}  // namespace

DriveController::DriveController(IMotor& left, IMotor& right, IClock& clock,
                                 IEstopInput& estop_input, const DriveParams& params)
    : motors_{&left, &right}, clock_(clock), estop_input_(estop_input), params_(params) {}

DriveTick DriveController::tick() {
  const int64_t now = clock_.nowUs();
  const bool pressed = estop_input_.pressed();
  bool restart_escs = false;
  float setpoints[kSideCount];
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Hardware e-stop: latch on the first pressed sample; the trip ends only after
    // the input reads released for the debounce time, then the ESCs restart.
    if (pressed) {
      estop_released_since_us_ = -1;
      if (!hasTripLocked(StopSource::kHardware)) {
        setTripBitLocked(StopSource::kHardware, true);
        latchLocked(StopSource::kHardware);
      }
    } else if (hasTripLocked(StopSource::kHardware)) {
      if (estop_released_since_us_ < 0) estop_released_since_us_ = now;
      if (now - estop_released_since_us_ >= kEstopReleaseDebounceUs) {
        setTripBitLocked(StopSource::kHardware, false);
        setTripBitLocked(StopSource::kEscRestart, true);
        restart_escs = true;
      }
    }
    if (!restart_escs && hasTripLocked(StopSource::kEscRestart) && !pressed &&
        motors_[0]->escReady() && motors_[1]->escReady()) {
      setTripBitLocked(StopSource::kEscRestart, false);
    }

    // Timeout: only wheel commands renew the deadline.
    if (state_.commanded && now >= command_deadline_us_) {
      state_.commanded = false;
      ++state_.timeout_count;
    }

    const float dt_s = last_tick_us_ < 0 ? 0.0f : static_cast<float>(now - last_tick_us_) * 1e-6f;
    last_tick_us_ = now;
    const float max_step = params_.max_wheel_accel * dt_s;
    for (int i = 0; i < kSideCount; ++i) {
      if (state_.estop_latched) {
        state_.setpoint[i] = 0.0f;  // immediate, no ramp
        continue;
      }
      const bool drive = state_.commanded && !state_.held_until_zero;
      const float target = drive ? command_[i] : 0.0f;
      state_.setpoint[i] = params_.max_wheel_accel > 0.0f
                               ? slewToward(state_.setpoint[i], target, max_step)
                               : target;
    }
    std::copy(std::begin(state_.setpoint), std::end(state_.setpoint), setpoints);
  }

  if (restart_escs) {
    motors_[0]->beginEscRestart(now);
    motors_[1]->beginEscRestart(now);
  }
  return {motors_[0]->update(setpoints[0], now), motors_[1]->update(setpoints[1], now)};
}

CommandResult DriveController::setWheelVelocities(float left, float right) {
  if (!std::isfinite(left) || !std::isfinite(right)) return CommandResult::kInvalid;
  const int64_t now = clock_.nowUs();
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_.estop_latched) return CommandResult::kEstopLatched;

  const float limit = std::max(params_.max_wheel_speed, 0.0f);
  command_[0] = std::clamp(left, -limit, limit);
  command_[1] = std::clamp(right, -limit, limit);
  command_deadline_us_ = now + static_cast<int64_t>(params_.cmd_timeout_ms) * 1000;
  state_.commanded = true;

  if (state_.held_until_zero) {
    if (std::fabs(left) > kZeroCommand || std::fabs(right) > kZeroCommand) {
      return CommandResult::kHeldUntilZero;
    }
    state_.held_until_zero = false;
  }
  return CommandResult::kAccepted;
}

void DriveController::stopForSessionLoss() {
  std::lock_guard<std::mutex> lock(mutex_);
  state_.commanded = false;
}

void DriveController::estop(StopSource source) {
  std::lock_guard<std::mutex> lock(mutex_);
  latchLocked(source);
}

ClearResult DriveController::clearEstop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!state_.estop_latched) return ClearResult::kNotLatched;
  if (state_.active_trips != 0) return ClearResult::kTripActive;
  state_.estop_latched = false;
  state_.first_source = StopSource::kNone;
  state_.source_mask = 0;
  state_.held_until_zero = true;
  return ClearResult::kCleared;
}

void DriveController::setTrip(StopSource trip, bool active) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (active && !hasTripLocked(trip)) latchLocked(trip);
  setTripBitLocked(trip, active);
}

void DriveController::setParams(const DriveParams& params) {
  std::lock_guard<std::mutex> lock(mutex_);
  params_ = params;
}

DriveState DriveController::state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

bool DriveController::isStopped() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_.setpoint[0] == 0.0f && state_.setpoint[1] == 0.0f &&
         (!state_.commanded || state_.held_until_zero ||
          (command_[0] == 0.0f && command_[1] == 0.0f));
}

void DriveController::latchLocked(StopSource source) {
  if (!state_.estop_latched) {
    state_.estop_latched = true;
    state_.first_source = source;
  }
  state_.source_mask |= bit(source);
  state_.commanded = false;
}

bool DriveController::hasTripLocked(StopSource trip) const {
  return (state_.active_trips & bit(trip)) != 0;
}

void DriveController::setTripBitLocked(StopSource trip, bool active) {
  if (active) {
    state_.active_trips |= bit(trip);
  } else {
    state_.active_trips &= static_cast<uint8_t>(~bit(trip));
  }
}

}  // namespace cb
