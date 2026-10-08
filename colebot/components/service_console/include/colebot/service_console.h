// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

namespace cb {

// Starts the USB service console (esp_console REPL on USB-Serial-JTAG) in its own
// low-priority task. Sprint 0 registers only `help`; the commands in PLAN.md §7.8
// come with sprint 1.
void startServiceConsole();

}  // namespace cb
