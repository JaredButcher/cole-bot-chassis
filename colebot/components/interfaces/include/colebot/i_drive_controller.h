// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>

namespace cb {

// Who entered the e-stop latch, and which automatic trips are active. Bit values
// for masks (PLAN.md §7.3).
enum class StopSource : uint8_t {
  kNone = 0,
  kRos = 1 << 0,         // /chassis/estop service
  kConsole = 1 << 1,     // service console
  kHardware = 1 << 2,    // trip: hardware e-stop pressed
  kEscRestart = 1 << 3,  // trip: ESCs restarting after their power returned
  kLowBattery = 1 << 4,  // trip: sprint 2
};

enum class CommandResult : uint8_t {
  kAccepted,
  kHeldUntilZero,  // accepted, but the drive stays stopped until a zero command (after a clear)
  kEstopLatched,   // rejected
  kInvalid,        // rejected: non-finite value
};

enum class ClearResult : uint8_t { kCleared, kNotLatched, kTripActive };

struct DriveParams {
  uint32_t cmd_timeout_ms = 250;
  float max_wheel_speed = 40.0f;  // rad/s; commands are clamped to this
  float max_wheel_accel = 50.0f;  // rad/s²; safety ramp. <= 0 disables the ramp
};

struct DriveState {
  bool estop_latched = false;
  StopSource first_source = StopSource::kNone;
  uint8_t source_mask = 0;   // every source since the latch was entered
  uint8_t active_trips = 0;  // trips still active (clear is refused while non-zero)
  bool commanded = false;    // a wheel command is current (not timed out, not latched)
  bool held_until_zero = false;
  float setpoint[2] = {0.0f, 0.0f};  // rad/s after the ramp, per Side
  uint32_t timeout_count = 0;
};

// Between the command sources (ROS, console) and the motors. All methods are
// safe to call from any task.
class IDriveController {
 public:
  virtual ~IDriveController() = default;
  // Wheel angular velocities (rad/s, wheel frame). Renews the command timeout.
  virtual CommandResult setWheelVelocities(float left, float right) = 0;
  // The agent session dropped: stop now instead of waiting for the timeout.
  virtual void stopForSessionLoss() = 0;
  virtual void estop(StopSource source) = 0;
  virtual ClearResult clearEstop() = 0;
  // Automatic trips other than the hardware e-stop (e.g. low battery). Activating
  // one enters the latch.
  virtual void setTrip(StopSource trip, bool active) = 0;
  virtual void setParams(const DriveParams& params) = 0;
  virtual DriveState state() const = 0;
  // Setpoints are zero: safe to change ESC configuration or write flash.
  virtual bool isStopped() const = 0;
};

}  // namespace cb
