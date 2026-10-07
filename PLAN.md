# Cole Bot Chassis Controller — Project Plan

## 1. Overview

Firmware for the chassis controller of a small differential-drive robot.

| Item            | Choice                                                                   |
| --------------- | ------------------------------------------------------------------------ |
| MCU             | ESP32-C6 (single-core RISC-V, 160 MHz)                                   |
| Framework       | ESP-IDF (CMake) + FreeRTOS                                               |
| Language        | C++ (C++17 or newer)                                                     |
| Drive           | 2 × ESC over DShot, one per side (left / right), driven independently    |
| Motor feedback  | Bidirectional DShot telemetry (eRPM) for wheel speed estimation          |
| DShot library   | [AlfredoSystems/AlfredoDShot](https://github.com/AlfredoSystems/AlfredoDShot) (v1.1), one GPIO per ESC |
| ESC firmware    | AM32 (≥ 2.21, updated as needed) in 3D (reversible) mode                 |
| Primary control | Custom binary protocol over a hardware UART (not the dev board's USB)    |
| Debug / service | Text CLI console over native USB (USB-Serial-JTAG), ESP32-C6-DevKitC-1   |

The firmware project lives in `colebot/` (currently the ESP-IDF hello-world template). Mechanical CAD lives in `model/`.

## 2. Roadmap

| Sprint       | Scope                                                                                     |
| ------------ | ----------------------------------------------------------------------------------------- |
| **0 — Setup** | Toolchain, project skeleton, library integration (prerequisite for sprint 1)              |
| **1 — Core**  | CLI console, UART control interface, motor control, motor feedback (RPM over time)       |
| 2            | Battery & system monitoring, telemetry reporting                                          |
| 3            | BLE control interface                                                                     |
| 4            | IMU driver                                                                                |
| 5            | Sensor fusion: motor feedback + IMU for odometry / movement tracking                      |

Sprints 2–5 are listed in rough order; we can reorder them later. The sprint 1 architecture leaves room for each of them (see §7).

## 3. Architecture

### 3.1 Task layout (FreeRTOS)

```
           USB (console)                    UART1 (control link)
                │                                   │
        ┌───────▼────────┐                 ┌────────▼─────────┐
        │   cli task     │                 │ control_link task│
        │ (esp_console)  │                 │ (protocol parse) │
        └───────┬────────┘                 └────────┬─────────┘
                │   commands / queries              │
                └──────────────┬────────────────────┘
                               ▼
                    ┌───────────────────────┐
                    │ drive command arbiter │  (source selection, timeout)
                    └──────────┬────────────┘
                               ▼  setpoint (L, R)
                    ┌───────────────────────┐         ┌──────────────────┐
                    │ motor task (periodic) │───────▶ │ feedback store   │
                    │ DShot TX + eRPM RX    │ samples │ (ring buffers)   │
                    └───────────────────────┘         └──────────────────┘
                                                            ▲
                                      cli / control_link read │
```

| Task             | Priority | Period / trigger            | Responsibility                                                                                  |
| ---------------- | -------- | --------------------------- | ----------------------------------------------------------------------------------------------- |
| `motor`          | highest  | fixed rate (e.g. 1 kHz)     | Send DShot frames to both ESCs, read eRPM, apply ramp/limits, failsafe, push feedback samples   |
| `control_link`   | high     | UART RX events              | Deframe and decode protocol messages, dispatch commands, send responses and streamed telemetry  |
| `cli`            | low      | blocking line input         | Interactive console for debug, arming/driving as the `CLI` source, config, live RPM view         |

Notes:
- The C6 has a single core, so the motor task must stay short and deterministic. It must not do any logging or blocking I/O.
- ESCs disarm when the frame stream stops, so the motor task sends frames continuously (zero throttle when idle).
- Tasks share data through small, clearly owned structures: a mutex- or atomic-protected setpoint, a FreeRTOS queue for commands, and lock-free ring buffers for feedback samples.

### 3.2 Design principles: interfaces and dependency injection

Wherever it's reasonable, the firmware is split into C++ classes that implement **interfaces**. Collaborators are passed in through the constructor, so any dependency can be swapped for a mock or fake in host tests.

- **Interfaces** are pure abstract classes:
  - named with an `I` prefix (`IEsc`, `IClock`, …)
  - a virtual destructor, only pure-virtual methods, no data members
  - **no ESP-IDF or FreeRTOS headers**, only `<cstdint>` and shared core types

  This is what lets every consumer compile on a PC.
- **Constructor injection by reference.** No singletons, globals, or service locators inside classes.
- **`app_main.cpp` is the composition root.** It creates every object (static storage, no heap after init), wires them together, and starts the tasks.
- **Concrete classes are either hardware-free or hardware-bound:**
  - hardware-free classes live in `core` and are host-tested
  - hardware-bound classes live in `platform`. They are thin adapters over ESP-IDF or AlfredoDShot and are verified on the robot.
- **Where not to add an interface.** Pure functions and small value types are tested directly with no indirection, because mocking them adds nothing. Examples: 3D throttle mapping, unit conversion, `RingBuffer<T, N>`, slew limiter, direction tracker, protocol messages. Interrupt handlers and the RMT internals also stay behind the `IEsc` adapter rather than being abstracted further.
- **Cost.** A virtual call per motor per 1 kHz tick is negligible. Interrupt-time code never goes through virtual dispatch.
- **Thread safety.** Each interface documents whether it may be called from multiple tasks. `IDriveController` and `IWheelFeedback` must allow it. Core classes use `std::mutex` / `std::atomic`. ESP-IDF maps them onto FreeRTOS, so the same code builds on the host and on the target without FreeRTOS headers.

**Sprint 1 interfaces:**

| Interface           | Responsibility                                                                 | Production impl (layer)                     | Mock (gMock)              | Fake                    |
| ------------------- | ------------------------------------------------------------------------------ | ------------------------------------------- | ------------------------- | ----------------------- |
| `IClock`            | Monotonic time in µs                                                           | `EspClock` (platform, `esp_timer`)          | `MockClock`               | `FakeClock`             |
| `IEsc`              | One DShot ESC: `begin`, `send(value)`, `command`, `isArmed`, telemetry status / eRPM / age, link stats, `echoPulses`, push-pull | `AlfredoEsc` (platform, AlfredoDShot) | `MockEsc` | — |
| `IMotor`            | Signed command → ESC each tick; inversion, direction tracking, returns a `WheelSample` | `Motor` (core, uses `IEsc`, `IClock`) | `MockMotor`               | —                       |
| `IDriveController`  | Arm / disarm / e-stop (with stop source) / clear, latch sources and active conditions, state-change counter, wheel or arcade (forward, turn) setpoints per `ControlSource`, arm-to-claim ownership, ramp, command lease / failsafe, state snapshot | `DriveController` (core, uses 2 × `IMotor`, `IClock`) | `MockDriveController` | — |
| `IWheelFeedback`    | Latest / filtered values, history, and link health per wheel                   | `FeedbackStore` (core)                      | `MockWheelFeedback`       | —                       |
| `IControlTransport` | Carries control-protocol bytes to and from the host: non-blocking `read` / `write`, `isConnected` | `UartControlTransport` (platform, UART1) | `MockControlTransport` | `FakeControlTransport` |
| `IConfigStore`      | Typed get / set / load / save of drive parameters                              | `NvsConfigStore` (platform, NVS)            | `MockConfigStore`         | `FakeConfigStore`       |

**Test double naming.** The name comes from the interface with the `I` dropped, plus a prefix that says what kind of double it is:
- **`Mock<Name>`** is a gMock class (`MOCK_METHOD`) used to set and verify expectations. Every interface has one.
- **`Fake<Name>`** is a small working in-memory implementation with test helpers. Examples: `FakeClock::advanceUs()`, `FakeControlTransport::injectRx()` / `takeTx()`, `FakeConfigStore` backed by a map. These exist only where a stateful stand-in is more useful than expectations. Add one when a test needs it.
- **Files:** `test/host/doubles/mock_<name>.h` and `fake_<name>.h` (snake_case), e.g. `mock_esc.h`, `fake_control_transport.h`. Interface headers follow the same pattern: `interfaces/include/colebot/i_<name>.h`.
- No other prefixes (`Stub`, `Dummy`, `InMemory`, `Test…`). If a test needs a canned return value, it uses a `Mock<Name>` with `ON_CALL`.

**Other core classes** that consume these interfaces:
- `ControlLink`: protocol framing and decoding over `IControlTransport`, dispatching to `IDriveController` and streaming from `IWheelFeedback`.
- `CliCommands`: command handlers over the same interfaces, writing to an output sink.

The FreeRTOS tasks (`MotorTask`, `ControlLinkTask`) are small platform wrappers that call `tick()` / `poll()` on core objects at their rate.

Later sprints follow the same pattern:
- `IBatteryMonitor` (sprint 2)
- `BleControlTransport` implementing `IControlTransport`, so `ControlLink` is reused unchanged (sprint 3)
- `IImu` (sprint 4)
- `IOdometry` / `IPoseEstimator` (sprint 5)

### 3.3 Component layout (proposed)

```
colebot/
├── CMakeLists.txt
├── sdkconfig.defaults           # target esp32c6, USB-Serial-JTAG console, UART, FreeRTOS tick, etc.
├── main/
│   └── app_main.cpp             # composition root: construct, wire, start tasks
├── components/
│   ├── board/                   # pin map, board constants, default drive parameters
│   ├── alfredo_dshot/           # AlfredoDShot (submodule) + IDF CMake wrapper + Arduino shim
│   ├── colebot_protocol/        # submodule: github.com/JaredButcher/colebot-protocol (protobuf + nanopb + framing)
│   ├── interfaces/              # header-only pure abstract interfaces (IClock, IEsc, IMotor, ...) — no IDF
│   ├── core/                    # hardware-free implementations + pure helpers — no IDF (host-tested)
│   │                            #   Motor, DriveController, FeedbackStore, ControlLink, CliCommands,
│   │                            #   throttle mapping, unit conversion, filters, RingBuffer, DirectionTracker
│   ├── platform/                # ESP-IDF implementations: AlfredoEsc, EspClock, UartControlTransport,
│   │                            #   NvsConfigStore, MotorTask / ControlLinkTask
│   └── cli/                     # esp_console + argtable glue -> core CliCommands
└── test/
    └── host/                    # plain-CMake host tests (see §5.5)
        ├── doubles/             # mock_<name>.h (every interface) + fake_<name>.h (where useful)
        └── ...                  # tests for core, built with interfaces + core + colebot_protocol
```

**Dependency rule:** `core` depends only on `interfaces`, `colebot_protocol`, and the C++ standard library. `platform` and `cli` depend on `core`, `interfaces`, and ESP-IDF. Nothing depends on `platform` except `main`. The host test build enforces this: if a `core` file includes an IDF header, it fails to compile.

## 4. Sprint 0 — Setup

- [ ] **ESP-IDF version.** AlfredoDShot requires Arduino core 3.x, which is built on ESP-IDF 5.x. It uses the RMT TX `io_loop_back` / `io_od_mode` flags to share one pad between TX and RX. Those flags are an ESP-IDF 5.x feature, so we should not move to 6.x without checking them. `.vscode/settings.json` points to v5.2.1 and the devcontainer uses `espressif/idf:latest`. Pin both to the same **5.5.x** release (the version Arduino core 3.3 uses).
- [ ] Set `IDF_TARGET=esp32c6` and add `sdkconfig.defaults`. Include `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` so the console runs on the DevKitC-1's native **USB** port.
- [ ] Replace the hello-world template with a C++ `app_main.cpp` (`extern "C" void app_main()`). Remove `pytest_hello_world.py` and `sdkconfig.ci`.
- [ ] Replace the root `.gitignore` (currently a Rust template) with an ESP-IDF one that ignores `build/`, `sdkconfig`, `sdkconfig.old`, `managed_components/`, and similar.
- [ ] **Integrate AlfredoDShot** (tasks A1–A4; full plan in [`docs/plans/alfredo-dshot.md`](docs/plans/alfredo-dshot.md)). Add the upstream repo unmodified as a submodule pinned to `v1.1`, plus a wrapper component with an IDF `CMakeLists.txt` and a minimal `compat/Arduino.h` shim. No fork.
- [ ] **Start `colebot-protocol`** (tasks P1–P5; full plan in [`docs/plans/colebot-protocol.md`](docs/plans/colebot-protocol.md)). The repo `github.com/JaredButcher/colebot-protocol` exists and is added as a submodule at `colebot/components/colebot_protocol` (relative URL, tracking `main` until `v0.1.0`). Clone this repo with `git clone --recurse-submodules`, or run `git submodule update --init` after cloning. This work needs no hardware, so it can run in parallel with the rest of sprint 0.
- [ ] Create the `interfaces/`, `core/`, `platform/`, and `cli/` component skeletons. Create the `test/host/` CMake project (GoogleTest + GoogleMock via `FetchContent`) with one passing placeholder test, so the dependency rule in §3.3 is enforced from day one.
- [ ] Define the pin map in `components/board/`: two DShot GPIOs, UART1 TX/RX, and spare pins reserved for I²C (IMU) and ADC (battery).

## 5. Sprint 1 — Core Features

### 5.1 Motor control

- One `Motor` (`IMotor`) per side, driving an `IEsc`. The production `IEsc` is `AlfredoEsc`, which wraps one `AlfredoDShot` instance: bidirectional, DSHOT600 by default (AM32's normal rate, and the rate where its telemetry matches the spec), plus a configurable motor magnet count.
- **Startup sequence** (in `app_main`, before the motor task starts):
  1. Call `AlfredoDShot::releaseBootloader()` for both pins (exposed as a static `AlfredoEsc` helper). This holds the lines low so AM32 leaves its bootloader after an ESP reset. Pass `holdMs = 0` for the first pin and the default 2500 ms for the second, so both are released in one 2.5 s window.
  2. Call `begin(pin, DSHOT600, true, poles)` for each ESC.
  3. Construct `Motor` and `DriveController` and start the motor task. During the first ~1.2 s, `send()` forces zero throttle on its own until AM32 arms. `isArmed()` reports when arming is done.
- **Frame loop:** `send(value)` sends one frame and collects the reply to the previous one, so the motor task calls it once per ESC per tick at a steady 1 kHz. It can busy-wait up to ~135 µs if called too soon, which never happens at 1 kHz.
- **Bidirectional driving.** A ground robot needs forward and reverse, so the ESCs run AM32 **3D mode**:
  - DShot value `0` = stop, `48..1047` = reverse (slow → fast), `1048..2047` = forward (slow → fast).
  - The library's `sendThrottle()` is one-directional (0..1), so `Motor` maps a signed command in `[-1.0, 1.0]` to these ranges itself. This is the same mapping as `throttle3D()` in the library's `Rotini_V4_Telemetry` example.
  - 3D mode and spin direction are ESC settings. Set them with the AM32 configurator, or with `DSHOT_CMD_3D_MODE_ON` / `DSHOT_CMD_SPIN_DIRECTION_*` followed by `DSHOT_CMD_SAVE_SETTINGS`, sent via `command()` while the motor is stopped.
- Per-side inversion. Prefer the ESC's own spin-direction setting so that "forward" means forward at the ESC. Also keep a firmware inversion flag as a fallback.
- `DriveController` (`IDriveController`):
  - Accepts setpoints as either per-wheel commands `(left, right)` or as arcade `(forward, turn)` with differential mixing (all normalized -1..1 in sprint 1).
  - Applies slew-rate limiting (ramp) and output clamping.
  - **Failsafe = command lease.** `Arm` and each motion command from the active source grant a lease. When it expires, the robot **disarms**: the claim is released and output ramps to zero. Setpoint commands get a lease equal to the failsafe timeout (default 250 ms), so hosts resend them (about 20 Hz). Only motion commands renew the lease; pings, queries, and other traffic don't. See [the protocol plan](docs/plans/colebot-protocol.md#4-schema-v1-draft).
  - Arm/disarm state: `Disarm` ramps output down to zero.
  - E-stop latch:
    - Entered by an `EStop` from any source, or by an automatic trip. Output goes to zero immediately, with no ramp.
    - Records the first source plus an accumulated bitmask of all sources.
    - `ClearEStop` is accepted from any source, but refused while an automatic trip is still active. After clearing, an explicit `Arm` is required.
- **Control ownership** (inside `DriveController`, keyed by `ControlSource`). `Arm` claims the robot and `Disarm` releases it. See [the protocol plan](docs/plans/colebot-protocol.md#4-schema-v1-draft) for the full rules.
  - `Arm` makes the sender the active source. It is refused while the robot is armed by another source, so a takeover is always `Disarm` then `Arm`.
  - `Disarm` is accepted from any source and clears the active source.
  - Motion commands are only accepted from the active source. Others get `Nack(NOT_ARMED)` or `Nack(NOT_ACTIVE_SOURCE)`.
  - **Invariant: armed ⇔ claimed.** Lease expiry is an automatic `Disarm`: the claim is released and `State` reports `last_release_reason = LEASE_EXPIRED`. An e-stop also releases the claim. To stay armed while stationary, the owner streams zero setpoints.
  - The CLI is a source like any other. There is no default owner at boot.
- Sprint 1 is open-loop (throttle) control. Closed-loop speed control (PID on measured RPM) is a stretch goal. The design should allow `DriveController` to accept speed setpoints later.

### 5.2 Motor feedback

- After each `send()`, read `status()`, `erpm()` / `rpm()`, and `ageUs()` for each ESC.
  - `status() == DSHOT_RX_OK` means a valid reading. Note that eRPM `0` is a real "not spinning" value, not a lost frame.
  - The other status values (`NO_REPLY`, `FRAMING`, `BAD_GCR`, `BAD_CRC`) mark the sample invalid.
- Convert to physical units:
  - motor RPM = eRPM × 2 / magnet count (the library's `rpm()`)
  - wheel RPM = motor RPM / drive reduction (the chain/sprocket reduction)
  - wheel speed (m/s) = wheel RPM × π × wheel diameter / 60
- **Direction:** telemetry RPM is unsigned. When the command changes sign at speed, AM32 first slows the motor down and then reverses it. So the sign cannot simply follow the command. Track a per-wheel "actual direction" that flips only once measured RPM drops below a small threshold after a sign change.
- Store timestamped samples (`esp_timer_get_time()`) in a ring buffer per wheel, holding a few seconds of history.
- Provide a light low-pass or moving-average filter and a validity flag (stale or no telemetry → invalid).
- Expose: latest value, filtered value, and history (for CLI display and for streaming over the control link).
- Track link health per motor using the library's `stats()` / `lossPercent()` (sent, ok, no-reply, framing, bad GCR, bad CRC).

**Drive parameters.** These are configurable at runtime (`config set`) and persisted in NVS. Compile-time defaults live in `components/board/`:

| Parameter          | Default                     | Notes                                                   |
| ------------------ | --------------------------- | ------------------------------------------------------- |
| `motor_poles`      | 14                          | Magnet count (12N14P outrunner); passed to `begin()`    |
| `drive_reduction`  | 1.0                         | Motor revs per wheel rev; placeholder until measured    |
| `wheel_diameter_m` | 0.1524                      | 6 in HiGrip wheel                                       |
| `dshot_mode`       | DSHOT600                    | Applied at boot                                         |
| `dshot_push_pull`  | off                         | `setPushPull()`; depends on ESC/board wiring            |
| `invert_left` / `invert_right` | false / false   | Firmware fallback; prefer ESC spin-direction setting    |

Changing `motor_poles` or `dshot_mode` re-initializes the ESC channels, so it is only allowed while disarmed.

### 5.3 CLI console (dev board USB)

- Built on `esp_console` (REPL + argtable3) using `esp_console_new_repl_usb_serial_jtag()`. It runs on the DevKitC-1's native **USB** port (USB-Serial-JTAG, GPIO 12/13), and the same cable also provides JTAG debugging.
- ESP_LOG output shares this console. When no host is attached, USB-Serial-JTAG writes can stall until they time out. That's fine for the `cli` and `control_link` tasks, and one more reason the motor task never logs.
- Initial command set:

| Command                          | Purpose                                              |
| -------------------------------- | ---------------------------------------------------- |
| `status`                         | Arm state, active source, setpoints, RPM, link stats |
| `disarm` / `estop` / `clear-estop` | Motor state control (any source). There is no standalone `arm`: a lone arm would expire after 250 ms, so `drive` claims for itself |
| `drive <left> <right> [--for S]` | Arms as the `CLI` source, streams the per-wheel command (-1.0..1.0) at 20 Hz, then disarms at the end or on any key. Refused if another source owns the robot |
| `rpm [--watch] [--hz N]`         | Print RPM / wheel speed, optionally streaming        |
| `rpm dump [--seconds N]`         | Dump RPM history as CSV for plotting                 |
| `dshot cmd <side> <cmd>`         | Send DShot special command (beacon, 3D mode, direction, save, EDT on/off) |
| `dshot diag [side]`              | `echoPulses()`, `status()`, link stats / loss %, armed state |
| `config get\|set\|save <key> [value]` | Runtime parameters (ramp, timeout, poles, reduction, wheel size) |
| `link stats`                     | Control-link counters (frames, CRC errors, timeouts) |
| `sys`                            | Heap, task list/stack high-water marks, uptime       |

### 5.4 UART control interface (custom protocol)

**`colebot-protocol` repo (submodule, shared across projects).** The full plan is in [`docs/plans/colebot-protocol.md`](docs/plans/colebot-protocol.md). In summary:
- **Messages:** Protobuf (proto3). nanopb 0.4.9.2 on the MCU, with the runtime vendored and static allocation only. Python `protobuf` on hosts.
- **Framing:** `COBS( Envelope ‖ CRC-16/CCITT-FALSE ) 0x00`. `Envelope` = `protocol_version`, `seq`, and a `oneof payload`.
- **Message set v1:**
  - `Ping`, `GetVersion`, `Arm`, `Disarm`, `EStop` / `ClearEStop`
  - `SetWheelCommand`, `SetArcadeCommand`, `GetState`, `ConfigureStream`
  - replies: `Ack` / `Nack`, `Pong`, `Version`, `State`, streamed `WheelFeedback`
- **Language support:** C++17 framing + nanopb glue with no heap, exceptions, or RTTI. Python package with a `Client` and the `colebot-ctl` tool, which is used for the sprint 1 UART acceptance tests.
- **Builds** as an ESP-IDF component, a plain CMake library, or a pip package. Generated code is committed.
- **Testing:** golden vectors shared by the C++ and Python test suites.

**Bandwidth check.** A `WheelFeedback` frame is about 30–40 bytes. At 100 Hz that is ~4 kB/s, which fits within 115200 baud (~11.5 kB/s). Higher rates or more fields need a faster baud rate.

**Firmware side:**
- `ControlLink` (core) does framing, decoding, dispatch, and streaming. It only sees `IControlTransport`, `IDriveController`, `IWheelFeedback`, and `IClock`.
- `UartControlTransport` (platform, `IControlTransport`) uses UART1 with the IDF UART driver and event queue. Baud rate is configurable and persisted (default 115200, with 921600+ available), since the host isn't fixed yet.
- `ControlLinkTask` feeds incoming bytes to the parser, and `ControlLink` dispatches messages to the arbiter and drive controller.
- Sends periodic telemetry at the configured rate.
- Counts link statistics, which are visible from the CLI.

### 5.5 Testing

Sprint 1 has host-side unit tests for both the protocol and the hardware-free firmware logic. Hardware-dependent code is verified on the robot.

- **`colebot-protocol`** owns its own tests, inside that repo.
  - C++ framing + nanopb (GoogleTest, plain CMake): round-trip for every message, corrupted CRC, truncated and partial frames, garbage between frames, and oversize frames.
  - Python (pytest): the same round-trip and framing cases.
  - **Cross-language golden vectors:** frames encoded by Python must decode in C++, and the reverse. This catches mismatches between the two implementations.
- **`core`** is tested from `colebot/test/host/`, a plain CMake project that compiles `interfaces` + `core` + `colebot_protocol`. `doubles/` holds the `Mock<Name>` and `Fake<Name>` classes from §3.2.
  - **Unit tests, with collaborators mocked:**
    - `Motor`, using `MockEsc` + `FakeClock`: 3D mapping at the edges (0, ±min, ±1.0, NaN), inversion, waiting for arming, direction tracking through a reversal, invalid telemetry status
    - `DriveController`, using `MockMotor` + `FakeClock`: ramp limits, lease expiry (and that only motion commands renew it), e-stop from several sources (first source + accumulated bitmask), `ClearEStop` refused while a condition is active, ownership (`Arm` claims, repeated `Arm` from the owner is idempotent, `Arm` from another source refused, `Disarm` from any source releases, motion from a non-owner refused and doesn't renew the lease, the armed ⇔ claimed invariant holds after every transition, `Arm` starts a lease, lease expiry disarms with `LEASE_EXPIRED`, e-stop clears the claim with `ESTOP`), arcade mixing, e-stop latch / clear, disarmed rejection
    - `FeedbackStore`: history, filtering, staleness, unit conversion with non-default poles / reduction / wheel size
    - `ControlLink`, using `FakeControlTransport` + `MockDriveController` + `MockWheelFeedback`: every message dispatches correctly, `Nack` on bad input, `Ping` does not renew the lease, `State` pushed on change with an incrementing count, periodic `State` at `state_hz`, stream rate, CRC-error counting
    - `CliCommands`: argument parsing and output against mocked interfaces
  - **Pure helpers** (throttle mapping, conversions, `RingBuffer`, filters) are tested directly.
  - **A few wiring tests** use real core objects with only the edges faked (`MockEsc`, `FakeClock`, `FakeControlTransport`). Example: a `SetWheelCommand` frame in, then no further motion command for 250 ms, must show the ESCs ramping to zero.
- **Framework:** GoogleTest + GoogleMock via CMake `FetchContent`, the same in both places. Plain CMake rather than the ESP-IDF `linux` target, so the tests also run on Windows and don't need an IDF install.
- **CI** (when the repo has one): build the firmware for `esp32c6` and run both host test suites.

### 5.6 Sprint 1 acceptance criteria

- [ ] Both motors spin forward and reverse independently from the CLI and from the UART protocol.
- [ ] Losing the control link, or the host stopping its setpoint stream, **disarms** the robot once the lease expires. The host sees `State` `CHANGE` with `last_release_reason = LEASE_EXPIRED`. A host that keeps pinging but stops sending commands is also disarmed. A lone `Arm` with no following setpoint disarms after the lease.
- [ ] Ownership: after the UART host arms, CLI `drive` is refused. CLI `disarm` stops the robot, and the host is notified through a `State` `CHANGE` (`DISARM_COMMAND` by `CLI`). CLI `drive` can then claim the robot while the host's `Arm` and commands get `Nack(NOT_ACTIVE_SOURCE)`.
- [ ] `estop` stops the motors immediately from either interface. An e-stop from the CLI is pushed to the UART host as a `State` (`CHANGE`) frame, with `first_latch_source = CLI`.
- [ ] Live wheel RPM is visible in the CLI and streamed over the control link. An RPM history dump plots cleanly. Measured speed matches a hand or tachometer check within a reasonable tolerance.
- [ ] `colebot_protocol` builds and passes its unit tests on a host PC, independent of this firmware.
- [ ] `core` host tests pass, covering the cases listed in §5.5. All hardware access goes through the interfaces in §3.2.
- [ ] The motor task holds its loop period with no missed deadlines while the CLI and control link are busy.

## 6. Hardware notes and constraints

- **RMT channels.** The ESP32-C6 RMT has **2 TX + 2 RX channels**. Two bidirectional DShot motors use all four. That leaves no RMT for anything else, including the dev board's addressable RGB LED, which normally uses RMT. A status LED would need another method (e.g. a plain GPIO LED, or SPI-driven WS2812).
- **One GPIO per ESC.** AlfredoDShot puts an RMT TX channel (open-drain) and an RX channel on the same pad, and the ESC replies on that same wire.
- **Signal wiring is out of scope** for this software (pull-up and series resistors are a hardware concern). The firmware only exposes what depends on it: the `dshot_push_pull` option and the `dshot diag` check (`echoPulses()` = 31 means the line is driven and released cleanly).
- **Push-pull caveat.** If push-pull is enabled, the line is released from the TX-done ISR. Flash writes (e.g. NVS saves) can delay that ISR and lose a reply, so `config save` is only allowed while disarmed.
- **ESC firmware.** The ESCs run **AM32**, updated as needed: ≥ 2.21 is assumed so EDT current reads in 1 A steps. AM32 picks bidirectional mode by itself when the line idles high. 3D mode must be enabled.
- **AM32 configurator.** The library's `AM32_ConfiguratorLink` example needs USB-OTG (TinyUSB), which the C6 doesn't have. ESC configuration has to happen through a separate AM32 linker or by sending DShot commands from this firmware.
- **Pin choice.** Keep the DShot pins off the C6 strapping pins (GPIO 4, 5, 8, 9, 15) and the USB pins (GPIO 12/13). The DShot lines idle high, which would affect boot mode on strapping pins.
- **UARTs.** The C6 has UART0, UART1, and LP_UART. The console uses USB-Serial-JTAG and the control link uses UART1. UART0 is free, but on the DevKitC-1 its default pins (GPIO 16/17) are wired to the on-board USB-UART bridge, so leave those pins alone or use UART0 only through that port. LP_UART is spare.
- **Pins to reserve now:** I²C SDA/SCL for the IMU and ADC pins for battery voltage/current sense (sprint 2+).

## 7. Future sprints (high level)

### Sprint 2 — Battery & system monitoring / telemetry
- Battery voltage and current via ADC (calibrated with `esp_adc_cali`), and/or via AM32 **Extended DShot Telemetry**:
  - Enable it with `DSHOT_CMD_EDT_ENABLE`. The library then exposes `temperatureC()`, `voltage()`, and `current()`.
  - Voltage and temperature update at ~5 Hz, current at ~25 Hz.
  - `current()` is in 1 A steps and assumes AM32 ≥ 2.21.
- `IBatteryMonitor` (ADC and/or EDT implementations). Low-voltage warning and cutoff integrate with `IDriveController`.
- System health: heap, task stacks, CPU load, reset reason, ESC temperatures.
- New protocol messages: `BatteryStatus`, `SystemStatus`. Matching CLI commands.

### Sprint 3 — BLE control interface
- NimBLE GATT service carrying the same `colebot_protocol` messages over a characteristic. `BleControlTransport` implements `IControlTransport`, so `ControlLink` is reused unchanged.
- Add BLE as another source for the command arbiter, with explicit priority rules.
- Watch flash and RAM budget, and coexistence with the motor task timing.

### Sprint 4 — IMU
- `IImu` interface with an I²C (or SPI) driver implementation and task, with calibration of gyro bias and accelerometer offsets.
- Timestamped samples using the same time base as wheel feedback.

### Sprint 5 — Motion estimation (odometry + IMU fusion)
- Wheel odometry from RPM history (differential-drive kinematics). Track width becomes a config parameter.
- Fuse it with IMU yaw rate (complementary filter first, EKF later if needed) to estimate pose and velocity.
- Optional closed-loop velocity and heading control.
- New protocol messages: `Odometry` / `Pose`.

## 8. Decisions and open questions

### Decided

| Topic              | Decision                                                                                       |
| ------------------ | ---------------------------------------------------------------------------------------------- |
| DShot library      | AlfredoDShot v1.1, upstream submodule + IDF wrapper/shim                                       |
| ESC firmware       | AM32 (≥ 2.21, updated as needed), 3D mode                                                      |
| Signal wiring      | Out of scope for the software; firmware exposes `dshot_push_pull` and `dshot diag`            |
| Magnet count       | Default 14, configurable                                                                       |
| Drive reduction    | Default 1.0, configurable (set once the drivetrain is measured)                                |
| Dev board / console| ESP32-C6-DevKitC-1, CLI on the native USB port (USB-Serial-JTAG)                               |
| Control UART host  | Undecided / mixed (SBC, MCU, PC): protocol repo builds without ESP-IDF; C++ and Python clients |
| Project directory  | Keep `colebot/`                                                                                |
| Code structure     | Classes implement pure-abstract interfaces (where reasonable); constructor injection; `app_main` is the composition root; `core` is IDF-free |
| Naming             | Interfaces `I<Name>` in `i_<name>.h`; test doubles `Mock<Name>` (gMock, every interface) and `Fake<Name>` (in-memory, where useful) |
| Testing            | Host tests (GoogleTest + GoogleMock, plain CMake) for `colebot_protocol` and `core`, with a `Mock<Name>` for every interface and `Fake<Name>` where useful |
| Protocol repo      | `github.com/JaredButcher/colebot-protocol`, submodule at `components/colebot_protocol`         |
| Message definitions| Protobuf: nanopb on the MCU, `protobuf` on hosts, own COBS + CRC-16 framing, generated code committed |
| License            | GPL-3.0-or-later (required by AlfredoDShot); SPDX header `GPL-3.0-or-later` on new source files |
| Protocol license   | GPL-3.0-or-later                                                                               |
| nanopb             | Runtime 0.4.9.2 vendored in `colebot-protocol/third_party/nanopb`; generator is a pinned dev dependency |
| Drive command units| Wheel and arcade commands normalized −1..1; a velocity command in real units comes with closed-loop control |
| E-stop / disarm    | Shared latch (command + automatic trips), first source + bitmask; any source may clear, `Nack` while a trip is active; `Disarm` ramps, `EStop` immediate |
| Control ownership  | Armed ⇔ claimed. `Arm` claims for the sender and starts a lease; `Arm` from a non-owner while armed → `Nack`; `Disarm` from any source, lease expiry, or e-stop releases; motion only from the owner |
| Submodule plans    | [`docs/plans/colebot-protocol.md`](docs/plans/colebot-protocol.md), [`docs/plans/alfredo-dshot.md`](docs/plans/alfredo-dshot.md) |

### Open

1. **Goal commands** (`RotateDegrees`, `MoveMeters`, …): deferred to sprint 5, once odometry and IMU fusion exist. The default position is to **keep this controller a setpoint executor**: the host runs goal-level behaviour in a closed loop using the streamed odometry, as a ROS base controller does with `cmd_vel`. Reasons to add goal commands here later:
   - the host is not real-time, or the link is lossy (BLE)
   - the chassis has the best high-rate pose estimate (fused IMU + wheels) for precise turns

   If they are added, they fit the lease model as bounded goals (see the protocol plan), so nothing in sprint 1 blocks them.

Values that will be measured later, such as drive reduction, are configurable with placeholder defaults.
