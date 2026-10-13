// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <gmock/gmock.h>

#include "colebot/i_clock.h"

namespace cb {

class MockClock : public IClock {
 public:
  MOCK_METHOD(int64_t, nowUs, (), (const, override));
};

}  // namespace cb
