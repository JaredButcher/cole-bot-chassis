// SPDX-License-Identifier: GPL-3.0-or-later
#include "colebot/motor.h"

#include <algorithm>

#include "colebot/throttle_3d.h"

namespace cb {

namespace {
int signOf(float value) { return value > 0.0f ? 1 : (value < 0.0f ? -1 : 0); }
}  // namespace

Motor::Motor(IEsc& esc, const EscConfig& esc_config, const MotorParams& params)
    : esc_(esc), esc_config_(esc_config), params_(params) {}

MotorParams Motor::paramsSnapshot() const {
  std::lock_guard<std::mutex> lock(params_mutex_);
  return params_;
}

void Motor::setParams(const MotorParams& params) {
  std::lock_guard<std::mutex> lock(params_mutex_);
  params_ = params;
}

bool Motor::computeReady() const { return restart_ == Restart::kNone && esc_.isArmed(); }

bool Motor::escReady() const { return ready_.load(); }

void Motor::beginEscRestart(int64_t now_us) {
  esc_.holdLineLow();
  restart_ = Restart::kHoldingLow;
  restart_start_us_ = now_us;
  ready_ = false;
  loop_.reset();
}

WheelSample Motor::update(float velocity_cmd, int64_t now_us) {
  const MotorParams params = paramsSnapshot();
  const float dt_s =
      last_update_us_ < 0 ? 0.0f : static_cast<float>(now_us - last_update_us_) * 1e-6f;
  last_update_us_ = now_us;

  float throttle = 0.0f;  // wheel frame
  if (restart_ == Restart::kHoldingLow) {
    // The line is held low; nothing is sent until the hold is over.
    if (now_us - restart_start_us_ >= kRestartHoldUs) {
      esc_.begin(esc_config_);
      restart_ = Restart::kWaitingForArm;
    }
  }
  if (restart_ == Restart::kHoldingLow) {
    measure(0, now_us, dt_s, params);
  } else {
    if (restart_ == Restart::kWaitingForArm && esc_.isArmed()) restart_ = Restart::kNone;
    if (computeReady()) {
      throttle = runVelocityLoop(velocity_cmd, dt_s, params);
    } else {
      loop_.reset();
    }
    const float esc_throttle = params.invert ? -throttle : throttle;
    esc_.send(throttleTo3dDshot(esc_throttle));
    measure(signOf(esc_throttle), now_us, dt_s, params);
  }

  sample_.time_us = now_us;
  sample_.throttle = throttle;
  ready_ = computeReady();
  sample_.esc_ready = ready_;
  sample_.link = esc_.linkStats();
  return sample_;
}

float Motor::runVelocityLoop(float velocity_cmd, float dt_s, const MotorParams& params) {
  const VelocityLoopGains gains{params.kv, params.ks, params.kp, params.ki, params.min_speed};
  return loop_.update(velocity_cmd, sample_.velocity, dt_s, gains);
}

void Motor::measure(int esc_direction_cmd, int64_t now_us, float dt_s,
                    const MotorParams& params) {
  const EscTelemetry telemetry = esc_.telemetry();
  sample_.status = telemetry.status;
  if (telemetry.status == EscRxStatus::kOk) {
    speed_ = erpmToWheelSpeed(telemetry.erpm, params.motor_poles, params.drive_reduction);
    last_valid_us_ = now_us;
  } else if (last_valid_us_ < 0 ||
             now_us - last_valid_us_ > static_cast<int64_t>(params.telemetry_hold_us)) {
    speed_ = 0.0f;
  }
  sample_.valid = last_valid_us_ >= 0 &&
                  now_us - last_valid_us_ <= static_cast<int64_t>(params.telemetry_hold_us);

  const int esc_direction = direction_.update(esc_direction_cmd, speed_, params.direction_flip_speed);
  const float velocity = static_cast<float>(params.invert ? -esc_direction : esc_direction) * speed_;

  sample_.raw_velocity = velocity;
  sample_.position += static_cast<double>(velocity) * dt_s;
  const float tau = std::max(params.velocity_filter_tau_s, 0.0f);
  const float alpha = (tau + dt_s) > 0.0f ? dt_s / (tau + dt_s) : 1.0f;
  sample_.velocity += alpha * (velocity - sample_.velocity);
}

}  // namespace cb
