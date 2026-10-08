// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "colebot/i_estop_input.h"

namespace cb {

class FakeEstopInput : public IEstopInput {
 public:
  bool pressed() const override { return pressed_; }
  void press() { pressed_ = true; }
  void release() { pressed_ = false; }

 private:
  bool pressed_ = false;
};

}  // namespace cb
