// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <gmock/gmock.h>

#include "colebot/i_agent_link.h"

namespace cb {

class MockAgentLink : public IAgentLink {
 public:
  MOCK_METHOD(bool, probe, (NetIface iface, uint32_t timeout_ms), (override));
  MOCK_METHOD(bool, open, (), (override));
  MOCK_METHOD(void, close, (), (override));
  MOCK_METHOD(bool, ping, (uint32_t timeout_ms), (override));
};

}  // namespace cb
