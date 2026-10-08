// SPDX-License-Identifier: GPL-3.0-or-later
#include "colebot/service_console.h"

#include "esp_console.h"
#include "esp_err.h"

namespace colebot {

void startServiceConsole() {
  esp_console_repl_t* repl = nullptr;
  esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
  repl_config.prompt = "colebot>";
  repl_config.task_priority = 2;  // below the ros and motor tasks (PLAN.md §3.2)

  esp_console_dev_usb_serial_jtag_config_t hw_config =
      ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));
  ESP_ERROR_CHECK(esp_console_register_help_command());
  ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

}  // namespace colebot
