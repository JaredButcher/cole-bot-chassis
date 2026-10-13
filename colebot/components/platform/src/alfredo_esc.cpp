// SPDX-License-Identifier: GPL-3.0-or-later
#include "colebot/alfredo_esc.h"

#include "AlfredoDShot.h"

namespace cb {

namespace {

DShotMode toLibrary(DshotMode mode) {
  switch (mode) {
    case DshotMode::k150: return DSHOT150;
    case DshotMode::k300: return DSHOT300;
    case DshotMode::k600: return DSHOT600;
    case DshotMode::k1200: return DSHOT1200;
  }
  return DSHOT600;
}

EscRxStatus fromLibrary(DShotRxStatus status) {
  switch (status) {
    case DSHOT_RX_OK: return EscRxStatus::kOk;
    case DSHOT_RX_IDLE: return EscRxStatus::kIdle;
    case DSHOT_RX_NO_REPLY: return EscRxStatus::kNoReply;
    case DSHOT_RX_FRAMING: return EscRxStatus::kFraming;
    case DSHOT_RX_BAD_GCR: return EscRxStatus::kBadGcr;
    case DSHOT_RX_BAD_CRC: return EscRxStatus::kBadCrc;
  }
  return EscRxStatus::kIdle;
}

}  // namespace

AlfredoEsc::AlfredoEsc(int gpio) : gpio_(gpio), dshot_(new AlfredoDShot()) {}

AlfredoEsc::~AlfredoEsc() {
  dshot_->end();
  delete dshot_;
}

void AlfredoEsc::releaseBootloader(std::initializer_list<int> gpios) {
  // Pull every line low at once (holdMs = 0), and wait only on the last one,
  // so all ESCs share one 2.5 s window.
  if (gpios.size() == 0) return;
  const int* last = gpios.end() - 1;
  for (const int* gpio = gpios.begin(); gpio != gpios.end(); ++gpio) {
    AlfredoDShot::releaseBootloader(*gpio, gpio == last ? 2500 : 0);
  }
}

bool AlfredoEsc::begin(const EscConfig& config) {
  dshot_->setPushPull(config.push_pull);
  return dshot_->begin(gpio_, toLibrary(config.mode), true, config.motor_poles);
}

void AlfredoEsc::end() { dshot_->end(); }

bool AlfredoEsc::send(uint16_t value) { return dshot_->send(value); }

void AlfredoEsc::command(EscCommand cmd, uint8_t repeat) {
  dshot_->command(static_cast<uint16_t>(cmd), repeat);
}

bool AlfredoEsc::commandPending() const { return dshot_->commandPending(); }

bool AlfredoEsc::isArmed() const { return dshot_->isArmed(); }

EscTelemetry AlfredoEsc::telemetry() const {
  return {fromLibrary(dshot_->status()), dshot_->erpm(), dshot_->ageUs()};
}

EscLinkStats AlfredoEsc::linkStats() const {
  const auto& s = dshot_->stats();
  return {s.sent, s.ok, s.noReply, s.framing, s.badGcr, s.badCrc};
}

void AlfredoEsc::resetLinkStats() { dshot_->resetStats(); }

uint16_t AlfredoEsc::echoPulses() const { return dshot_->echoPulses(); }

void AlfredoEsc::holdLineLow() {
  dshot_->end();
  AlfredoDShot::releaseBootloader(gpio_, 0);  // pulls low and returns; the caller times the hold
}

}  // namespace cb
