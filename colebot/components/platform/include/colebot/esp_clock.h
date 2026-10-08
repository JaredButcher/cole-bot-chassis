// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "colebot/i_clock.h"

namespace cb {

// IClock over esp_timer (µs since boot).
class EspClock final : public IClock {
 public:
  int64_t nowUs() const override;
};

}  // namespace cb
