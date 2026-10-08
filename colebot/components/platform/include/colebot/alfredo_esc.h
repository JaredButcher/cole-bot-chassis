// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <initializer_list>

#include "colebot/i_esc.h"

// Only alfredo_esc.cpp includes AlfredoDShot.h (docs/plans/alfredo-dshot.md §2),
// so the library and its Arduino shim stay out of every other translation unit.
class AlfredoDShot;

namespace colebot {

// IEsc over one AlfredoDShot instance, bidirectional, on one GPIO.
class AlfredoEsc final : public IEsc {
 public:
  explicit AlfredoEsc(int gpio);
  ~AlfredoEsc() override;
  AlfredoEsc(const AlfredoEsc&) = delete;
  AlfredoEsc& operator=(const AlfredoEsc&) = delete;

  // Releases every pin from the AM32 bootloader in one 2.5 s window. Call once at
  // startup, before begin(), for all ESC pins.
  static void releaseBootloader(std::initializer_list<int> gpios);

  bool begin(const EscConfig& config) override;
  void end() override;
  bool send(uint16_t value) override;
  void command(EscCommand cmd, uint8_t repeat = 6) override;
  bool commandPending() const override;
  bool isArmed() const override;
  EscTelemetry telemetry() const override;
  EscLinkStats linkStats() const override;
  void resetLinkStats() override;
  uint16_t echoPulses() const override;
  void holdLineLow() override;

 private:
  int gpio_;
  AlfredoDShot* dshot_;  // owned; heap-allocated once at construction
};

}  // namespace colebot
