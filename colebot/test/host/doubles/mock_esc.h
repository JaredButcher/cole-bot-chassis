// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <gmock/gmock.h>

#include "colebot/i_esc.h"

namespace colebot {

class MockEsc : public IEsc {
 public:
  MOCK_METHOD(bool, begin, (const EscConfig& config), (override));
  MOCK_METHOD(void, end, (), (override));
  MOCK_METHOD(bool, send, (uint16_t value), (override));
  MOCK_METHOD(void, command, (EscCommand cmd, uint8_t repeat), (override));
  MOCK_METHOD(bool, commandPending, (), (const, override));
  MOCK_METHOD(bool, isArmed, (), (const, override));
  MOCK_METHOD(EscTelemetry, telemetry, (), (const, override));
  MOCK_METHOD(EscLinkStats, linkStats, (), (const, override));
  MOCK_METHOD(void, resetLinkStats, (), (override));
  MOCK_METHOD(uint16_t, echoPulses, (), (const, override));
  MOCK_METHOD(void, holdLineLow, (), (override));
};

}  // namespace colebot
