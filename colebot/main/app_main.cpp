// SPDX-License-Identifier: GPL-3.0-or-later
//
// Composition root (PLAN.md §3.3): creates every object, wires them together and
// starts the tasks. Sprint 0 only brings up the service console; the motor, network
// and ROS tasks are added in sprint 1.
#include "colebot/service_console.h"
#include "esp_app_desc.h"
#include "esp_log.h"

namespace {
constexpr const char* kTag = "colebot";
}  // namespace

extern "C" void app_main() {
  ESP_LOGI(kTag, "colebot chassis controller %s", esp_app_get_description()->version);
  cb::startServiceConsole();
}
