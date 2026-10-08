// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <gmock/gmock.h>

#include "colebot/i_network_interface.h"

namespace cb {

class MockNetworkInterface : public INetworkInterface {
 public:
  MOCK_METHOD(void, start, (), (override));
  MOCK_METHOD(void, stop, (), (override));
  MOCK_METHOD(bool, linkUp, (), (const, override));
  MOCK_METHOD(bool, hasIp, (), (const, override));
  MOCK_METHOD(void, makeDefault, (), (override));
};

}  // namespace cb
