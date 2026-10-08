// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>

namespace colebot {

enum class DshotMode : uint8_t { k150, k300, k600, k1200 };
enum class EscRxStatus : uint8_t { kOk, kIdle, kNoReply, kFraming, kBadGcr, kBadCrc };
enum class EscCommand : uint16_t {  // values match the DShot spec
  kBeacon1 = 1, kSpinDirection1 = 7, kSpinDirection2 = 8, k3dModeOff = 9, k3dModeOn = 10,
  kSaveSettings = 12, kEdtEnable = 13, kEdtDisable = 14,
  kSpinDirectionNormal = 20, kSpinDirectionReversed = 21 };

struct EscConfig    { DshotMode mode; uint8_t motor_poles; bool push_pull; };
struct EscTelemetry { EscRxStatus status; uint32_t erpm; uint32_t age_us; };
struct EscLinkStats { uint32_t sent, ok, no_reply, framing, bad_gcr, bad_crc; };

// One bidirectional DShot ESC. Called only from the motor task.
class IEsc {
 public:
  virtual ~IEsc() = default;
  virtual bool begin(const EscConfig& config) = 0;   // (re)initialize; only while stopped
  virtual void end() = 0;
  virtual bool send(uint16_t value) = 0;             // 0 = stop, 48..2047 throttle; true = fresh telemetry
  virtual void command(EscCommand cmd, uint8_t repeat = 6) = 0;
  virtual bool commandPending() const = 0;
  virtual bool isArmed() const = 0;                  // AM32 arming window elapsed
  virtual EscTelemetry telemetry() const = 0;
  virtual EscLinkStats linkStats() const = 0;
  virtual void resetLinkStats() = 0;
  virtual uint16_t echoPulses() const = 0;           // wiring check, 31 = good
  virtual void holdLineLow() = 0;                    // end() and pull the line low until the next begin() (AM32 bootloader release)
};

}  // namespace colebot
