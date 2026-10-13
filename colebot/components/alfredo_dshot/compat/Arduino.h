// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Minimal Arduino shim so AlfredoDShot builds under plain ESP-IDF.
// Provides only what AlfredoDShot v1.1 uses. Do not grow this into a general Arduino layer.
#include <cstdint>
#include <cmath>
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ESP_ARDUINO_VERSION_VAL(major, minor, patch) (((major) << 16) | ((minor) << 8) | (patch))
#define ESP_ARDUINO_VERSION ESP_ARDUINO_VERSION_VAL(3, 3, 0)  // AlfredoDShot only checks >= 3.0.0

static inline void delay(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }
