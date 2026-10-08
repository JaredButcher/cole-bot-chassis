// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <gmock/gmock.h>

#include "colebot/i_estop_input.h"

namespace cb {

class MockEstopInput : public IEstopInput {
 public:
  MOCK_METHOD(bool, pressed, (), (const, override));
};

}  // namespace cb
