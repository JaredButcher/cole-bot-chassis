// SPDX-License-Identifier: GPL-3.0-or-later
#include "colebot/esp_clock.h"

#include "esp_timer.h"

namespace cb {

int64_t EspClock::nowUs() const { return esp_timer_get_time(); }

}  // namespace cb
