# AlfredoDShot Integration — Sprint 1 Plan

How the [AlfredoDShot](https://github.com/AlfredoSystems/AlfredoDShot) library is brought into the ESP-IDF firmware as a git submodule, adapted without forking, and wrapped behind the `IEsc` interface.

| Item           | Value                                                                        |
| -------------- | ---------------------------------------------------------------------------- |
| Upstream       | `github.com/AlfredoSystems/AlfredoDShot`                                     |
| Pinned version | **v1.1** (commit `8f667d0`, released 2026-10-05)                             |
| Mounted at     | `colebot/components/alfredo_dshot/AlfredoDShot` (submodule, unmodified)      |
| License        | **GPL-3.0**, which is why this repo is GPL-3.0-or-later (§7)             |
| Target         | ESP32-C6, ESP-IDF 5.5.x (upstream only documents ESP32-S3 + Arduino core 3.x) |

## 1. What the library needs from Arduino

A source review of v1.1 found only a small Arduino surface:

| Use                                                  | Where                    | Provided by the shim as                              |
| ---------------------------------------------------- | ------------------------ | ---------------------------------------------------- |
| `#include <Arduino.h>`                               | `AlfredoDShot.h`         | The shim file itself                                 |
| `#error` unless `ESP_ARDUINO_VERSION >= 3.0.0`       | `AlfredoDShot.h`         | `ESP_ARDUINO_VERSION_VAL()` + `ESP_ARDUINO_VERSION` = 3.3.0 |
| `delay(ms)`                                          | `releaseBootloader()`    | `vTaskDelay(pdMS_TO_TICKS(ms))`                      |
| `uint8_t`/`uint16_t`/…                               | throughout               | `<cstdint>`                                          |
| `NAN`                                                | EDT fields               | `<cmath>`                                            |
| `IRAM_ATTR`                                          | RMT callbacks            | `esp_attr.h`                                         |

Everything else is plain ESP-IDF: `driver/rmt_tx.h`, `driver/rmt_rx.h`, `driver/gpio.h`, `esp_timer.h`, `hal/gpio_ll.h`. There is no `Serial`, `String`, `pinMode`, or `micros()` in the library sources. Those appear only in the examples, which we don't build.

## 2. Wrapper component

```
colebot/components/alfredo_dshot/
├── AlfredoDShot/            # submodule @ v1.1, never edited
├── compat/
│   └── Arduino.h            # shim, see below
└── CMakeLists.txt
```

**`CMakeLists.txt`**
```cmake
idf_component_register(
  SRCS         "AlfredoDShot/src/AlfredoDShot.cpp"
  INCLUDE_DIRS "AlfredoDShot/src" "compat"
  REQUIRES     esp_driver_rmt esp_driver_gpio esp_timer
  PRIV_REQUIRES hal soc freertos)
# Third-party code: don't let its warnings fail our build.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error)
```

**`compat/Arduino.h`**
```cpp
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
```

**Containment rules**
- `compat/` is on the include path of anything that includes `AlfredoDShot.h`. Only the `platform` component (`AlfredoEsc`) does that. `core`, `interfaces`, and host tests never see this shim or AlfredoDShot.
- `AlfredoEsc.cpp` is the only file that includes `AlfredoDShot.h`.

## 3. ESP32-C6 compatibility review

The upstream README only covers the ESP32-S3. This table records what was checked statically and what still needs hardware.

| Concern                                      | C6 status                                                                    | Verified by       |
| -------------------------------------------- | ---------------------------------------------------------------------------- | ----------------- |
| RMT clock: library assumes 80 MHz            | `RMT_CLK_SRC_DEFAULT` on the C6 is PLL_F80M → 80 MHz                          | Build + echo test |
| RMT memory: `SOC_RMT_MEM_WORDS_PER_CHANNEL`  | 48 on the C6; the library reads the macro                                    | Build             |
| Channel count: 1 TX + 1 RX per ESC           | The C6 has 2 TX + 2 RX → exactly 2 ESCs, nothing left over                    | `begin()` succeeds for both |
| TX `io_loop_back` + `io_od_mode` flags       | Present in IDF 5.5; may warn as deprecated. **Re-check before any move to IDF 6.x** | Build             |
| `gpio_ll_od_enable/disable(&GPIO, pin)` (push-pull mode) | Available in the C6 HAL                                          | Build; push-pull test only if used |
| Busy-wait in `send()` (up to ~135 µs)        | Single core. Fine at 1 kHz, and the motor task is the highest priority       | Loop-timing check |
| Interrupt latency vs. ~25 µs AM32 turnaround (push-pull only) | Flash writes can stall the ISR → `save_params` / `config save` only while stopped | Bring-up B5 |

## 4. `IEsc` and `AlfredoEsc`

`IEsc` lives in `interfaces/` and must not include `AlfredoDShot.h`, because that header pulls in ESP-IDF. So the interface defines its own small types, and `AlfredoEsc` maps them.

```cpp
// interfaces/include/colebot/i_esc.h
namespace colebot {

enum class DshotMode : uint8_t { k150, k300, k600, k1200 };
enum class EscRxStatus : uint8_t { kOk, kIdle, kNoReply, kFraming, kBadGcr, kBadCrc };
enum class EscCommand : uint16_t {  // values match the DShot spec
  kBeacon1 = 1, kSpinDirection1 = 7, kSpinDirection2 = 8, k3dModeOff = 9, k3dModeOn = 10,
  kSaveSettings = 12, kEdtEnable = 13, kEdtDisable = 14,
  kSpinDirectionNormal = 20, kSpinDirectionReversed = 21 };

struct EscConfig    { DshotMode mode; uint8_t motor_poles; bool push_pull; };
struct EscTelemetry { EscRxStatus status; uint32_t erpm; uint32_t age_us; };
struct EscLinkStats { uint32_t sent, ok, no_reply, framing, bad_gcr, bad_crc; };

class IEsc {
 public:
  virtual ~IEsc() = default;
  virtual bool begin(const EscConfig& config) = 0;   // (re)initialize; only while stopped
  virtual void end() = 0;
  virtual bool send(uint16_t value) = 0;             // 0 = stop, 48..2047 throttle; true = fresh telemetry
  virtual void command(EscCommand cmd, uint8_t repeat = 6) = 0;
  virtual bool commandPending() const = 0;
  virtual bool isArmed() const = 0;                  // AM32 arming window elapsed
  virtual EscTelemetry telemetry() const = 0;
  virtual EscLinkStats linkStats() const = 0;
  virtual void resetLinkStats() = 0;
  virtual uint16_t echoPulses() const = 0;           // wiring check, 31 = good
  virtual void restart(uint32_t hold_ms) = 0;        // after ESC power loss: end(), hold low, begin() with the last config
};

}  // namespace colebot
```

- **`AlfredoEsc`** (platform) is constructed with its GPIO. `begin()` maps `EscConfig` onto `AlfredoDShot::begin(pin, mode, true, poles)` + `setPushPull()`. The other methods are one-line forwards with enum mapping.
- `static void AlfredoEsc::releaseBootloader(std::initializer_list<gpio_num_t> pins)` releases every pin in one 2.5 s window: it calls the library with `holdMs = 0` for all but the last pin.
- `restart(hold_ms)` is for the hardware e-stop (`PLAN.md` §7.3): the ESCs lose power while the ESP32 keeps running. It runs `end()`, then the same bootloader release as at startup for this pin (`hold_ms` works like the library's `holdMs`, so the caller overlaps both ESCs' holds: 0 for the first, 2500 for the second), then `begin()` with the last `EscConfig`.
- `MockEsc` (gMock) mirrors `IEsc` 1:1. There is no `FakeEsc` in sprint 1; add one only if a test needs ESC state, e.g. simulated eRPM that follows the throttle.
- The 3D signed-throttle mapping is **not** in `IEsc`. It's a pure helper in `core` used by `Motor`, so it's tested without any double.
- EDT getters (temperature, voltage, current) are left out of `IEsc` until sprint 2 needs them. Adding methods to an interface is cheap.

## 5. Hardware bring-up (sprint 0 → early sprint 1)

This runs before `Motor` and `DriveController` exist, so it uses a Kconfig option `COLEBOT_DSHOT_BRINGUP`. That makes `app_main` run a scripted sequence through `AlfredoEsc` and log the results, instead of starting the normal tasks. Once the service console's `dshot diag` covers B2–B3, the option is kept only for the B7 calibration sweep.

| Step | Check                                                                                         | Pass criteria                                           |
| ---- | --------------------------------------------------------------------------------------------- | ------------------------------------------------------- |
| B1   | `releaseBootloader()` both pins, `begin()` both ESCs                                          | Both return true; all 4 RMT channels allocated          |
| B2   | 1 kHz zero-throttle loop, log `echoPulses()` per ESC                                          | 31 on both                                              |
| B3   | Wait for `isArmed()`, log `telemetry().status`                                                | Armed within ~1.5 s; status `kOk` with eRPM 0 at rest   |
| B4   | 3D-mode check: forward ~10 % for 2 s, stop, reverse ~10 % for 2 s (motors unloaded)           | Correct directions; RPM plausible; loss < 1 %           |
| B5   | Both ESCs at 1 kHz for 60 s while logging from another task                                   | No missed loop deadlines; loss < 1 % per ESC            |
| B6   | `command(k3dModeOn)` + `command(kSaveSettings)` while stopped, then power-cycle the ESC        | 3D mode persists (only needed if not set via configurator) |
| B7   | Calibration sweep, wheels off the ground: step throttle both directions, log steady wheel speed per step | Data gives `kv`, `ks` and the minimum speed for the velocity loop (`PLAN.md` §7.2) |
| B8   | With the ESP32 running and the loop at zero throttle, press the hardware e-stop (ESC power off), release it, watch for ~3 s, then call `restart()` | Record whether the ESCs re-arm **without** `restart()` (does AM32 stay in its bootloader?). With `restart()` both arm and report `kOk` |

Signal wiring (pull-ups, series resistors) is out of scope for the software. If B2 or B3 fails, the result is reported as a hardware issue, with the README's troubleshooting table as the reference.

## 6. Upgrade and upstream policy

- **Pinned to a tag.** The library is young (v1.0 → v1.1 within days), so every bump is a deliberate PR that:
  - reviews the upstream diff
  - re-runs the §1 Arduino-surface check (new `Arduino.h` uses would break the shim build, which is the desired signal)
  - re-runs bring-up steps B2–B5 on hardware
- **Upstream contribution (optional, recommended).** Offer a PR to AlfredoDShot that:
  - guards the Arduino bits with `#ifdef ARDUINO`, using `esp_rom_delay_us`/`vTaskDelay` otherwise
  - adds an ESP-IDF `CMakeLists.txt`

  Once that is merged and tagged, delete `compat/` and point the wrapper at the upstream CMake.

## 7. License

AlfredoDShot is **GPL-3.0**. To match, this firmware repo has been **relicensed from MIT to GPL-3.0-or-later**:
- `LICENSE` now holds the GPL-3.0 text.
- The README states the license and the AlfredoDShot dependency.

What this means in practice:
- Any firmware binary handed out must come with, or offer, the complete corresponding source. Keeping the repo public covers this.
- New source files carry `// SPDX-License-Identifier: GPL-3.0-or-later`.
- The license is no longer a reason to replace AlfredoDShot. `IEsc` still keeps such a swap local if one is ever wanted for technical reasons.
- micro-ROS is linked into the same binary. It is Apache-2.0, which is compatible with GPL-3.0.

## 8. Work breakdown

| #  | Task                                                                           | Done when                                                  |
| -- | ------------------------------------------------------------------------------ | ---------------------------------------------------------- |
| A1 | Add submodule at `components/alfredo_dshot/AlfredoDShot`, checkout `v1.1`      | `.gitmodules` committed, submodule at `8f667d0`            |
| A2 | Wrapper `CMakeLists.txt` + `compat/Arduino.h`                                  | `idf.py set-target esp32c6 build` succeeds                 |
| A3 | `IEsc` + types in `interfaces/`; `MockEsc` in `test/host/doubles/`             | Host test project compiles a trivial `MockEsc` test        |
| A4 | `AlfredoEsc` in `platform/` (+ `releaseBootloader` helper)                     | Builds; used by the bring-up mode                          |
| A5 | `COLEBOT_DSHOT_BRINGUP` mode, run B1–B8 on hardware                            | All pass criteria met, results recorded in the PR          |
| A6 | (Optional) upstream PR for `#ifdef ARDUINO` + IDF CMake                        | PR opened                                                  |

A1–A4 need no hardware. A5 needs the robot (ESCs on AM32 with 3D mode, motors unloaded).
