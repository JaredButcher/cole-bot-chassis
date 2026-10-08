# Cole Bot Chassis Controller — Project Plan

## 1. Overview

Firmware for the chassis controller of a small differential-drive robot. The controller is a **micro-ROS node**: a ROS 2 computer runs the drive kinematics and odometry with `ros2_control`, and the ESP32 drives the two wheels as joints.

| Item            | Choice                                                                   |
| --------------- | ------------------------------------------------------------------------ |
| MCU             | ESP32-C6 (single-core RISC-V, 160 MHz)                                   |
| Framework       | ESP-IDF 5.5.x (CMake) + FreeRTOS                                         |
| Language        | C++ (C++17 or newer)                                                     |
| Drive           | 2 × ESC over DShot, one per side (left / right), driven independently    |
| Motor feedback  | Bidirectional DShot telemetry (eRPM) for wheel speed and position        |
| DShot library   | [AlfredoSystems/AlfredoDShot](https://github.com/AlfredoSystems/AlfredoDShot) (v1.1), one GPIO per ESC |
| ESC firmware    | AM32 (≥ 2.21, updated as needed) in 3D (reversible) mode                 |
| Host interface  | **micro-ROS** (Micro XRCE-DDS) over UDP to a micro-ROS agent             |
| ROS 2 distro    | **Jazzy** for now; upgrade to **Lyrical Luth** once micro-ROS for ESP-IDF supports it (§9) |
| Network         | W5500 SPI Ethernet (preferred) + built-in WiFi; one active at a time (§5) |
| Host            | Linux computer running ROS 2, the micro-ROS agent and `ros2_control`     |
| Debug / service | Minimal USB service console (USB-Serial-JTAG), ESP32-C6-DevKitC-1         |

Layout:
- `colebot/`: the firmware (currently the ESP-IDF hello-world template)
- `ros/`: the host-side ROS 2 packages
- `model/`: mechanical CAD

**What ROS provides instead of custom code.** We use standard ROS 2 packages wherever possible:
- `diff_drive_controller` handles kinematics, odometry, velocity and acceleration limits, `cmd_vel` timeout and TF.
- `joint_state_topic_hardware_interface` connects ros2_control to the firmware's topics.
- `twist_mux` combines command sources.
- `robot_localization` fuses wheel odometry and the IMU.
- The `ros2` CLI, rqt, rosbag and PlotJuggler cover operation, tuning and logging.

This replaces the earlier plan's custom protocol (`colebot-protocol`), UART link, BLE link, Python client, command ownership and on-device odometry.

## 2. Roadmap

| Sprint        | Scope                                                                                       |
| ------------- | ------------------------------------------------------------------------------------------- |
| **0 — Setup** | Toolchain, project skeleton, AlfredoDShot and micro-ROS integration, `ros/` workspace skeleton |
| **1 — Core**  | Motor control with wheel velocity loop, wheel feedback, networking + connection manager, micro-ROS node, host ros2_control bringup, `ros2 colebot` CLI, service console |
| 2             | Battery & system monitoring (`BatteryState`, diagnostics)                                    |
| 3             | IMU driver (`sensor_msgs/Imu`)                                                               |
| 4             | Localization on the host (`robot_localization` EKF: wheel odometry + IMU), Nav2 readiness    |
| —             | Lyrical upgrade, when upstream is ready (§9)                                                 |

The old BLE sprint is gone: micro-ROS has no BLE transport, and WiFi covers wireless operation.

## 3. Architecture

### 3.1 System overview

```
 Host (Linux, ROS 2 Jazzy)                                       ESP32-C6 (micro-ROS)
┌───────────────────────────────────────────────────┐           ┌────────────────────────────┐
│ teleop / Nav2 ─► twist_mux ─► diff_drive_controller│           │ /chassis node              │
│                                │  ▲  odom, TF      │           │  sub  wheel_commands       │
│        controller_manager (50 Hz)                  │           │  pub  wheel_states         │
│                ▼  │                                │  UDP      │  pub  diagnostics, estop_state│
│   joint_state_topic_hardware_interface             │◄────────► │  srv  estop, clear_estop,  │
│      pub /chassis/wheel_commands                   │ Ethernet  │       save_params          │
│      sub /chassis/wheel_states                     │  or WiFi  │  params (NVS-backed)       │
│                                                    │           │                            │
│ micro-ROS agent (UDP 8888) ◄──── DDS ────► rest of ROS graph   │ wheel velocity loop (1 kHz)│
└───────────────────────────────────────────────────┘           └────────────────────────────┘
```

The firmware never sees `cmd_vel`. It receives per-wheel angular velocity commands (rad/s) and reports per-wheel position (rad) and velocity (rad/s). Wheel geometry (radius, track width) lives only in the host's controller configuration.

### 3.2 Task layout (FreeRTOS)

```
   USB (service console)          Ethernet (W5500/SPI) / WiFi
          │                                  │  lwIP
  ┌───────▼────────┐               ┌─────────▼───────────┐
  │ console task   │               │ ros task            │
  │ (esp_console)  │               │ ConnectionManager + │
  └───────┬────────┘               │ rclc executor       │
          │ estop / status / net   └─────────┬───────────┘
          └──────────────┬───────────────────┘
                         ▼  wheel velocity setpoints, e-stop
              ┌───────────────────────┐
              │ drive controller      │  (command timeout, e-stop latch, ramp)
              └──────────┬────────────┘
                         ▼
              ┌───────────────────────┐         ┌──────────────────┐
              │ motor task (1 kHz)    │───────▶ │ feedback store   │──▶ wheel_states, diagnostics
              │ velocity loop, DShot  │ samples │ (latest, filter, │
              │ TX + eRPM RX          │         │  position)       │
              └───────────────────────┘         └──────────────────┘
```

| Task      | Priority                                 | Period / trigger         | Responsibility                                                                                   |
| --------- | ---------------------------------------- | ------------------------ | ------------------------------------------------------------------------------------------------ |
| `motor`   | highest (24, above IDF WiFi at 23)       | fixed 1 kHz              | Velocity loop, DShot frames to both ESCs, eRPM, position integration, ramp, push feedback samples |
| `ros`     | medium (below lwIP / WiFi / Ethernet)    | executor spin + timers   | Connection state machine, rclc executor, publish wheel states (after each command, idle timer otherwise), services, params |
| `console` | low                                      | blocking line input      | Service console for bring-up and recovery                                                         |

ESP-IDF also runs its own network tasks: lwIP `tcpip` (18), WiFi (23) and the W5500 RX task.

Notes:
- The C6 has a single core. The motor task must stay short and deterministic, with no logging or blocking I/O. It now shares the core with lwIP, WiFi and the SPI Ethernet driver. Loop timing is checked under network load in sprint 1 (§7.10).
- ESCs disarm when the frame stream stops, so the motor task sends frames continuously (zero throttle when idle).
- Tasks share data through small, clearly owned structures:
  - a mutex- or atomic-protected setpoint
  - a FreeRTOS queue for commands
  - an atomic or seqlock snapshot for the latest feedback

### 3.3 Design principles: interfaces and dependency injection

Wherever it's reasonable, the firmware is split into C++ classes that implement **interfaces**. Collaborators are passed in through the constructor, so any dependency can be swapped for a mock or fake in host tests.

- **Interfaces** are pure abstract classes:
  - named with an `I` prefix (`IEsc`, `IClock`, …)
  - a virtual destructor, only pure-virtual methods, no data members
  - **no ESP-IDF, FreeRTOS or micro-ROS headers**, only `<cstdint>` and shared core types

  This is what lets every consumer compile on a PC.
- **Constructor injection by reference.** No singletons, globals, or service locators inside classes.
- **`app_main.cpp` is the composition root.** It creates every object (static storage, no heap after init), wires them together, and starts the tasks.
- **Concrete classes are either hardware-free or hardware-bound:**
  - hardware-free classes live in `core` and are host-tested
  - hardware-bound classes live in `platform`. They are thin adapters over ESP-IDF, AlfredoDShot or micro-ROS and are verified on the robot.
- **The micro-ROS layer is a thin adapter.** `RosNode` (platform) owns the rclc objects and message buffers. It only converts between ROS messages and core calls. Decisions such as name matching, NaN rejection, the connection state machine and e-stop rules live in `core`.
- **Where not to add an interface.** Pure functions and small value types are tested directly with no indirection, because mocking them adds nothing. Examples: 3D throttle mapping, unit conversion, slew limiter, direction tracker, PI controller. Interrupt handlers and the RMT internals also stay behind the `IEsc` adapter rather than being abstracted further.
- **Cost.** A virtual call per motor per 1 kHz tick is negligible. Interrupt-time code never goes through virtual dispatch.
- **Thread safety.** Each interface documents whether it may be called from multiple tasks. `IDriveController` and `IWheelFeedback` must allow it. Core classes use `std::mutex` / `std::atomic`. ESP-IDF maps them onto FreeRTOS, so the same code builds on the host and on the target without FreeRTOS headers.

**Sprint 1 interfaces:**

| Interface           | Responsibility                                                                 | Production impl (layer)                     | Mock (gMock)              | Fake                    |
| ------------------- | ------------------------------------------------------------------------------ | ------------------------------------------- | ------------------------- | ----------------------- |
| `IClock`            | Monotonic time in µs                                                           | `EspClock` (platform, `esp_timer`)          | `MockClock`               | `FakeClock`             |
| `IEsc`              | One DShot ESC: `begin`, `send(value)`, `command`, `isArmed`, telemetry status / eRPM / age, link stats, `echoPulses`, push-pull | `AlfredoEsc` (platform, AlfredoDShot) | `MockEsc` | — |
| `IMotor`            | Wheel velocity setpoint (rad/s) → velocity loop → ESC each tick; inversion, direction tracking, position integration, ESC restart after power loss; returns a `WheelSample` | `Motor` (core, uses `IEsc`, `IClock`) | `MockMotor` | — |
| `IDriveController`  | Per-wheel velocity setpoints, command timeout, e-stop latch (source + trips), ramp limit, state snapshot | `DriveController` (core, uses 2 × `IMotor`, `IClock`, `IEstopInput`) | `MockDriveController` | — |
| `IEstopInput`       | Raw state of the hardware e-stop contact (`pressed()`)                        | `GpioEstopInput` (platform, GPIO)           | `MockEstopInput`          | `FakeEstopInput`        |
| `IWheelFeedback`    | Latest and filtered velocity, accumulated position, telemetry validity and link health per wheel | `FeedbackStore` (core)                      | `MockWheelFeedback`       | —                       |
| `IConfigStore`      | Typed get / set / load / save of drive parameters                              | `NvsConfigStore` (platform, NVS)            | `MockConfigStore`         | `FakeConfigStore`       |
| `INetworkInterface` | One network interface: `start` / `stop`, link up, has IP, make default route    | `EthW5500Interface`, `WifiStaInterface` (platform) | `MockNetworkInterface` | `FakeNetworkInterface` |
| `IAgentLink`        | micro-ROS session over the current default interface: `probe(timeout)` (discover or ping), `open`, `close`, `ping` | `MicroRosAgentLink` (platform) | `MockAgentLink` | `FakeAgentLink` |

**Test double naming.** The name comes from the interface with the `I` dropped, plus a prefix that says what kind of double it is:
- **`Mock<Name>`** is a gMock class (`MOCK_METHOD`) used to set and verify expectations. Every interface has one.
- **`Fake<Name>`** is a small working in-memory implementation with test helpers. Examples:
  - `FakeClock::advanceUs()`
  - `FakeNetworkInterface::setLink()`
  - `FakeAgentLink::agentAppears()`
  - `FakeConfigStore`, backed by a map

  These exist only where a stateful stand-in is more useful than expectations. Add one when a test needs it.
- **Files:** `test/host/doubles/mock_<name>.h` and `fake_<name>.h` (snake_case), e.g. `mock_esc.h`, `fake_agent_link.h`. Interface headers follow the same pattern: `interfaces/include/colebot/i_<name>.h`.
- No other prefixes (`Stub`, `Dummy`, `InMemory`, `Test…`). If a test needs a canned return value, it uses a `Mock<Name>` with `ON_CALL`.
- The host-side Python stand-in for the whole firmware is `fake_chassis` (§7.9), following the same convention.

**Other core classes** that consume these interfaces:
- `ConnectionManager`: the connecting / connected state machine (§5) over two `INetworkInterface`s and an `IAgentLink`. It tells `IDriveController` to stop when the session drops.
- `WheelCommandHandler`: validates a decoded wheel command (joint-name matching, finite values, clamping) and forwards it to `IDriveController`. `RosNode` decodes the `JointState` message and calls it.
- `DiagnosticsBuilder`: turns `IWheelFeedback`, `IDriveController` and connection state into key/value status entries. `RosNode` copies them into `diagnostic_msgs`.
- `ConsoleCommands`: service console handlers over the same interfaces, writing to an output sink.

The FreeRTOS tasks (`MotorTask`, `RosTask`) are small platform wrappers that call `tick()` / `poll()` on core objects at their rate.

Later sprints follow the same pattern:
- `IBatteryMonitor` (sprint 2)
- `IImu` (sprint 3)

### 3.4 Component layout

```
colebot/
├── CMakeLists.txt
├── sdkconfig.defaults           # target esp32c6, USB-Serial-JTAG console, FreeRTOS tick, lwIP, W5500, etc.
├── main/
│   └── app_main.cpp             # composition root: construct, wire, start tasks
├── components/
│   ├── board/                   # pin map, board constants, default drive parameters, joint names
│   ├── alfredo_dshot/           # AlfredoDShot (submodule) + IDF CMake wrapper + Arduino shim
│   ├── micro_ros_espidf_component/  # submodule, jazzy branch (pinned commit)
│   ├── interfaces/              # header-only pure abstract interfaces — no IDF, no micro-ROS
│   ├── core/                    # hardware-free implementations + pure helpers — no IDF (host-tested)
│   │                            #   Motor, DriveController, FeedbackStore, ConnectionManager,
│   │                            #   WheelCommandHandler, DiagnosticsBuilder, ConsoleCommands,
│   │                            #   throttle mapping, PI controller, unit conversion, filters, DirectionTracker
│   ├── platform/                # ESP-IDF / micro-ROS implementations: AlfredoEsc, EspClock, NvsConfigStore,
│   │                            #   EthW5500Interface, WifiStaInterface, MicroRosAgentLink, RosNode,
│   │                            #   MotorTask, RosTask
│   └── console/                 # esp_console + argtable glue -> core ConsoleCommands
├── app-colcon.meta              # micro-ROS build options (entity counts, transport, MTU)
└── test/
    └── host/                    # plain-CMake host tests (see §7.9)
        ├── doubles/             # mock_<name>.h (every interface) + fake_<name>.h (where useful)
        └── ...                  # tests for core, built with interfaces + core

ros/                             # colcon workspace source folder (host side)
├── colebot_description/         # URDF/xacro: base_link, wheel joints, imu_link; ros2_control tag (topic system | mock)
├── colebot_bringup/             # launch + config: agent, robot_state_publisher, controller_manager, controllers,
│                                #   twist_mux, diagnostic_aggregator; launch tests
├── colebot_cli/                 # `ros2 colebot …` command (ros2cli extension, §7.7)
└── colebot_fake_chassis/        # Python node implementing the firmware's topic contract (SIL tests, demos)
```

**Dependency rule:**
- `core` depends only on `interfaces` and the C++ standard library.
- `platform` and `console` depend on `core`, `interfaces`, ESP-IDF and (`platform` only) micro-ROS.
- Nothing depends on `platform` except `main`.

The host test build enforces this: if a `core` file includes an IDF or micro-ROS header, it fails to compile.

## 4. ROS interface

This is the contract between the firmware and the host. `fake_chassis` implements the same contract.

### 4.1 Names

| Item         | Value                                                                    |
| ------------ | ------------------------------------------------------------------------ |
| Node         | `chassis` (no namespace by default)                                       |
| Joint names  | `left_wheel_joint`, `right_wheel_joint` (compile-time constants in `board/`, must match the URDF) |
| Frames       | Firmware publishes none in sprint 1. `imu_link` in sprint 3                 |

### 4.2 Topics, services and parameters

| Name                        | Kind    | Type                                | Direction | Rate              | QoS                      | Notes |
| --------------------------- | ------- | ----------------------------------- | --------- | ----------------- | ------------------------ | ----- |
| `/chassis/wheel_commands`   | sub     | `sensor_msgs/JointState`            | host→MCU  | control rate (50 Hz) | best effort (matches the host's reliable publisher) | `name` + `velocity` (rad/s) only. Matched by name; NaN or unknown names rejected |
| `/chassis/wheel_states`     | pub     | `sensor_msgs/JointState`            | MCU→host  | after each command (= control rate); `state_idle_hz` when idle | best effort (host subscribes with `SensorDataQoS`) | `position` (rad, integrated on the MCU), `velocity` (rad/s, filtered) |
| `/chassis/estop_state`      | pub     | `std_msgs/Bool`                     | MCU→host  | on change + 1 Hz  | reliable                  | Details (first source, bitmask, active trips) are in diagnostics |
| `/diagnostics`              | pub     | `diagnostic_msgs/DiagnosticArray`   | MCU→host  | 1 Hz              | reliable                  | One `DiagnosticStatus` per message, rotating through groups, to stay under the MTU (§4.3) |
| `/chassis/estop`            | service | `std_srvs/Trigger`                  |           |                   |                           | Latch e-stop (source `ROS`) |
| `/chassis/clear_estop`      | service | `std_srvs/Trigger`                  |           |                   |                           | Refused while an automatic trip is active |
| `/chassis/save_params`      | service | `std_srvs/Trigger`                  |           |                   |                           | Write current parameters to NVS. Refused unless stopped |
| `/chassis` parameters       | params  | rclc parameter server (bool / int / double) |  |                  |                           | Listed below. `ros2 param get/set/list` |

Later sprints add `/chassis/battery` (`sensor_msgs/BatteryState`, 1 Hz, sprint 2) and `/chassis/imu` (`sensor_msgs/Imu`, 100 Hz, sprint 3).

**Parameters** (defaults in `components/board/`, persisted in NVS by `save_params` or the console):

| Parameter            | Default    | Notes                                                                  |
| -------------------- | ---------- | ---------------------------------------------------------------------- |
| `motor_poles`        | 14         | Magnet count (12N14P outrunner). Change only while stopped              |
| `drive_reduction`    | 1.0        | Motor revs per wheel rev; placeholder until measured                   |
| `invert_left` / `invert_right` | false / false | Firmware fallback; prefer the ESC spin-direction setting  |
| `dshot_mode`         | 600        | 150 / 300 / 600 / 1200. Change only while stopped                       |
| `dshot_push_pull`    | false      | Depends on ESC/board wiring                                             |
| `cmd_timeout_ms`     | 250        | Command timeout (§7.3)                                                  |
| `max_wheel_accel`    | TBD        | rad/s², a safety ramp limit. Normal limits live in `diff_drive_controller` |
| `kv`, `ks`           | TBD        | Feedforward: throttle per rad/s, and static-friction throttle (§7.2)    |
| `kp`, `ki`           | 0, 0       | PI gains; 0 = pure feedforward (open loop)                             |
| `state_idle_hz`      | 10         | `wheel_states` rate while no wheel commands arrive. Otherwise states are sent after each command (§7.5) |

Wheel radius and track width are **not** firmware parameters. They live in the host's `diff_drive_controller` configuration.

### 4.3 micro-ROS build limits

micro-ROS allocates everything statically. `colebot/app-colcon.meta` sets the entity counts, with headroom for later sprints:

| Setting                         | Value | Why                                                          |
| ------------------------------- | ----- | ------------------------------------------------------------ |
| `RMW_UXRCE_MAX_NODES`           | 1     |                                                              |
| `RMW_UXRCE_MAX_PUBLISHERS`      | 8     | wheel_states, estop_state, diagnostics, parameter events, battery, imu + spare |
| `RMW_UXRCE_MAX_SUBSCRIPTIONS`   | 2     | wheel_commands + spare                                       |
| `RMW_UXRCE_MAX_SERVICES`        | 10    | 3 own + the rclc parameter server's services + spare          |
| `RMW_UXRCE_TRANSPORT`           | udp   | Component default                                            |
| `RMW_UXRCE_MAX_TRANSPORT_MTU`   | 512 (default) | Largest message is `Imu` (~324 B). `Odometry` (~724 B) never crosses this link |

Variable-size fields (joint names, diagnostic strings) are preallocated with `micro_ros_utilities`.

### 4.4 Bandwidth

Estimates from the CDR layout, plus ~80 B per packet of micro-ROS/UDP/IP/Ethernet overhead:

| Stream                  | Payload | Rate   | On the wire |
| ----------------------- | ------- | ------ | ----------- |
| `wheel_states`          | ~105 B  | 50 Hz  | ~9.5 kB/s   |
| `wheel_commands`        | ~90 B   | 50 Hz  | ~8.5 kB/s   |
| `imu` (sprint 3)        | ~324 B  | 100 Hz | ~40 kB/s    |
| diagnostics, battery, estop_state | < 300 B | ~1 Hz | < 1 kB/s |
| **Total**               |         |        | **~60 kB/s (~0.5 Mbit/s), ~200 packets/s** |

Bytes are not a concern on either link. Packet rate and the CPU cost per packet are, because the network stack shares the motor task's single core. The control rate starts at 50 Hz and is one launch argument (§7.6). Doubling it to 100 Hz adds ~18 kB/s and 100 packets/s; the sprint 1 loop-timing test measures the headroom.

## 5. Networking and connection management

### 5.1 Interfaces

| Interface | Hardware / driver                                         | Addressing                               | Configured by                     |
| --------- | --------------------------------------------------------- | ---------------------------------------- | --------------------------------- |
| Ethernet  | W5500 on GP-SPI2: SCLK, MOSI, MISO, CS, INT, RST (6 GPIO). `esp_eth` W5500 driver | **Static IP** (point-to-point link to the onboard computer). Placeholder default `192.168.50.2/24`, computer at `192.168.50.1` | Console `net eth …`, NVS |
| WiFi      | Built-in 802.11ax, station mode                           | DHCP                                     | Console `net wifi <ssid> <pass>`, NVS |

The micro-ROS component's own network setup only brings up one of WiFi or Ethernet, so we don't use it. `EthW5500Interface` and `WifiStaInterface` set up `esp_netif` themselves, and micro-ROS uses only its UDP transport over whichever interface is the default route.

### 5.2 Connection state machine (`ConnectionManager`)

micro-ROS keeps one session with one agent, and the client always starts the connection. The robot therefore probes for an agent and commits to the first interface that finds one:

```
            boot
              │
              ▼
┌──────────────────────────┐  agent found on iface X   ┌───────────────────────────┐
│ CONNECTING               │──────────────────────────▶│ CONNECTED(X)              │
│ both interfaces started  │                           │ other interface stopped   │
│ each round: probe eth,   │◀──────────────────────────│ (esp_eth_stop/esp_wifi_stop)│
│ then wifi                │   session lost:           │ ping agent every 200 ms   │
└──────────────────────────┘   3 missed pings, or      └───────────────────────────┘
                               eth link-down (if X=eth)
                               → stop drive, tear down session, restart the other interface
```

Rules:
- **Probe.** Make the interface the default route, then run agent discovery (`rmw_uros_discover_agent`, ~500 ms timeout). If discovery finds nothing and a saved agent address exists for that interface, ping it. The agent runs with discovery enabled.
- **Ethernet first.** Each round probes Ethernet before WiFi. While the Ethernet link is physically up, a WiFi agent is not accepted until `eth_grace_ms` (default 5 s) after link-up. This stops a fast WiFi join from winning the boot race with the cable plugged in.
- **No switch back.** In `CONNECTED(wifi)`, Ethernet is stopped, so plugging in the cable does nothing until the WiFi session drops (or `net reconnect` forces it).
- **Losing the session.** Stop the drive first, then destroy the rclc entities and close the session. Restart the stopped interface and re-enter `CONNECTING`. A failover takes a few seconds. The robot is already stopped by then.
- **On connect.** Create the rclc entities, sync time (`rmw_uros_sync_session`, repeated every 60 s), load parameters, start publishing.
- **Wrong agent.** Discovery could find another robot's agent on a shared WiFi network. That risk is accepted for now (§10).

## 6. Sprint 0 — Setup

- [ ] **ESP-IDF version.** AlfredoDShot requires Arduino core 3.x, which is built on ESP-IDF 5.x. It uses the RMT TX `io_loop_back` / `io_od_mode` flags to share one pad between TX and RX. Those flags are an ESP-IDF 5.x feature, so we should not move to 6.x without checking them. `.vscode/settings.json` points to v5.2.1 and the devcontainer uses `espressif/idf:latest`. Pin both to the same **5.5.x** release (the version Arduino core 3.3 uses, and one micro-ROS is tested with).
- [ ] Set `IDF_TARGET=esp32c6` and add `sdkconfig.defaults`. Include `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` so the console runs on the DevKitC-1's native **USB** port.
- [ ] Replace the hello-world template with a C++ `app_main.cpp` (`extern "C" void app_main()`). Remove `pytest_hello_world.py` and `sdkconfig.ci`.
- [ ] Replace the root `.gitignore` (currently a Rust template) with one covering:
  - ESP-IDF: `build/`, `sdkconfig`, `sdkconfig.old`, `managed_components/`
  - colcon: `ros/build/`, `ros/install/`, `ros/log/`
- [x] **Remove the `colebot-protocol` submodule** (`colebot/components/colebot_protocol` and its `.gitmodules` entry).
- [ ] Archive the `colebot-protocol` repo on GitHub.
- [ ] **Integrate AlfredoDShot** (tasks A1–A4; full plan in [`docs/plans/alfredo-dshot.md`](docs/plans/alfredo-dshot.md)). Add the upstream repo unmodified as a submodule pinned to `v1.1`, plus a wrapper component with an IDF `CMakeLists.txt` and a minimal `compat/Arduino.h` shim. No fork.
- [ ] **Integrate micro-ROS:**
  - Add `micro_ros_espidf_component` as a submodule at `components/micro_ros_espidf_component`, pinned to a commit on its `jazzy` branch.
  - Add the build dependencies to the devcontainer's IDF Python environment: `pip install catkin_pkg colcon-common-extensions lark`. Build in a shell where ROS is **not** sourced.
  - Add `app-colcon.meta` (§4.3).
  - Note: the component's first build clones the micro-ROS sources by branch and caches them in the component directory. Builds are only reproducible while that cache is kept. Record the cloned commits in the PR that bumps the component.
  - Done when an int32 publisher example reaches a Jazzy agent over WiFi from the C6.
- [ ] **W5500 bring-up:** wire the module to GP-SPI2. Done when the C6 gets a link and IP over Ethernet and the int32 example reaches an agent over Ethernet.
- [ ] Create the `interfaces/`, `core/`, `platform/`, and `console/` component skeletons. Create the `test/host/` CMake project (GoogleTest + GoogleMock via `FetchContent`) with one passing placeholder test, so the dependency rule in §3.4 is enforced from day one.
- [ ] Define the pin map in `components/board/`:
  - two DShot GPIOs
  - W5500 SPI (SCLK, MOSI, MISO, CS, INT, RST)
  - hardware e-stop input (not a strapping pin)
  - spare pins reserved for I²C (IMU) and ADC (battery)
- [ ] **`ros/` workspace skeleton:**
  - `colebot_description` with a placeholder URDF
  - `colebot_bringup` with mock hardware, so `ros2 launch colebot_bringup bringup.launch.py use_mock_hardware:=true` brings up `diff_drive_controller` and responds to `cmd_vel`
  - a ROS devcontainer (`ros:jazzy`) or documented host setup
  - `colcon test` passing with `ament_lint_auto`
- [ ] Agent setup: Docker `microros/micro-ros-agent:jazzy` run with UDP 8888 and discovery enabled, started from the bringup launch file or a compose file.

## 7. Sprint 1 — Core Features

### 7.1 Motor control

- One `Motor` (`IMotor`) per side, driving an `IEsc`. The production `IEsc` is `AlfredoEsc`, which wraps one `AlfredoDShot` instance. It runs bidirectional at DSHOT600 by default (AM32's normal rate, and the rate where its telemetry matches the spec), with a configurable motor magnet count.
- **Startup sequence** (in `app_main`, before the motor task starts):
  1. Call `AlfredoDShot::releaseBootloader()` for both pins (exposed as a static `AlfredoEsc` helper). This holds the lines low so AM32 leaves its bootloader after an ESP reset. Pass `holdMs = 0` for the first pin and the default 2500 ms for the second, so both are released in one 2.5 s window.
  2. Call `begin(pin, DSHOT600, true, poles)` for each ESC.
  3. Construct `Motor` and `DriveController` and start the motor task. During the first ~1.2 s, `send()` forces zero throttle on its own until AM32 arms. `isArmed()` reports when arming is done.
  4. Start networking and the `ros` task. The motor task runs (sending zero throttle) whether or not an agent is connected.
- **Frame loop:** `send(value)` sends one frame and collects the reply to the previous one, so the motor task calls it once per ESC per tick at a steady 1 kHz. It can busy-wait up to ~135 µs if called too soon, which never happens at 1 kHz.
- **Bidirectional driving.** A ground robot needs forward and reverse, so the ESCs run AM32 **3D mode**:
  - DShot value `0` = stop, `48..1047` = reverse (slow → fast), `1048..2047` = forward (slow → fast).
  - The library's `sendThrottle()` is one-directional (0..1), so `Motor` maps a signed throttle in `[-1.0, 1.0]` to these ranges itself. This is the same mapping as `throttle3D()` in the library's `Rotini_V4_Telemetry` example.
  - 3D mode and spin direction are ESC settings. Set them with the AM32 configurator, or with `DSHOT_CMD_3D_MODE_ON` / `DSHOT_CMD_SPIN_DIRECTION_*` followed by `DSHOT_CMD_SAVE_SETTINGS`, sent from the console (`dshot cmd`) while the motor is stopped.
- Per-side inversion. Prefer the ESC's own spin-direction setting so that "forward" means forward at the ESC. Also keep a firmware inversion flag as a fallback.

### 7.2 Wheel velocity loop

`diff_drive_controller` sends wheel velocities in rad/s, so closed-loop speed control is part of sprint 1. Each `Motor` runs a velocity loop:

```
throttle = kv·ω_cmd + ks·sign(ω_cmd) + kp·e + ki·∫e      e = ω_cmd − ω_measured (filtered)
```

- A pure `PiController` helper (core) with output clamping and anti-windup (stop integrating while the output is saturated).
- With `kp = ki = 0` it is pure feedforward (open loop). That is the first bring-up step, so the drive works before tuning.
- **Calibrating `kv` / `ks`.** A bring-up sweep (AlfredoDShot plan step B7) steps the throttle and records steady wheel speed. Fit a line: the slope gives `kv` and the intercept gives `ks`.
- **Low speed.** Below some throttle AM32 won't start the motor, and eRPM telemetry gets coarse. Commands under a minimum speed (a parameter, from the sweep) are treated as zero. The integrator is reset at zero command.
- **Reversal.** Telemetry RPM is unsigned. The measured sign comes from the direction tracker (§7.4), so the loop doesn't fight AM32's slow-down-then-reverse behavior.

### 7.3 Drive controller and safety

`DriveController` (`IDriveController`) sits between the command sources and the motors:
- **Setpoints:** per-wheel angular velocity (rad/s), clamped to a maximum, with a safety ramp (`max_wheel_accel`).
- **Command timeout.** Each accepted `wheel_commands` message renews a deadline (`cmd_timeout_ms`, default 250 ms; 12 cycles at 50 Hz). When it expires, the setpoints ramp to zero.
  - Only wheel commands renew it.
  - The host must therefore publish every cycle: the hardware interface's `trigger_joint_command_threshold` must be `-1` (§7.6). Otherwise it stops publishing once the wheels reach speed, and the timeout fires.
  - Losing the agent session also stops the drive immediately (§5.2).
- **E-stop latch:**
  - Entered by the hardware e-stop (trip `HARDWARE`, below), the `estop` service (source `ROS`), the console (`CONSOLE`), or another automatic trip (`AUTO_*`, e.g. low battery in sprint 2). Output goes to zero immediately, with no ramp.
  - Records the first source, plus a bitmask of all sources and active trips. Published on `estop_state` and in diagnostics.
  - `clear_estop` (service or console) is refused while a trip is still active.
  - **After a clear, the drive stays stopped until a wheel command with both velocities ≈ 0 arrives.** This way a host still streaming an old non-zero command can't make the robot lurch.
- **Hardware e-stop input.** The physical e-stop cuts ESC power. An auxiliary contact also goes to a GPIO (`IEstopInput`), so the firmware and ROS know about it:
  - **Fail-safe polarity.** The input reads "released" only while the contact is closed, e.g. a normally-closed contact to GND with a pull-up: low = released, high = pressed. A broken wire or unplugged connector reads as pressed.
  - `DriveController` samples it every motor tick (1 kHz). The first "pressed" sample activates the `HARDWARE` trip, with no debounce on entry. The trip ends only after the input has read "released" continuously for 20 ms.
  - **Releasing the button doesn't restart anything.** The latch stays until `clear_estop`, and then the zero-command rule applies. This follows the usual e-stop rule that resetting the stop must not restart motion by itself.
  - **ESC restart.** While the button is pressed the ESCs are unpowered, so their telemetry reads `NO_REPLY`. Diagnostics report this as "unpowered (hardware e-stop)", not as a link fault. When power returns, AM32 boots again, but the ESP32 has kept running, so the startup bootloader release (§7.1) has to be repeated: when the `HARDWARE` trip ends, each `Motor` restarts its ESC (`IEsc::restart`: `end()`, hold the line low, `begin()`, wait for `isArmed()`). The two holds overlap, as at startup. An `ESC_RESTART` trip stays active until both ESCs are armed again, so `clear_estop` is refused until then. Bring-up step B8 checks whether AM32 really needs this.
    - The hold blocks for up to 2.5 s (the library uses `delay()`). It runs in the motor task, which has nothing else to do while the ESCs restart. The e-stop input is sampled again before `ESC_RESTART` ends, so a second press during the restart is still caught.
  - The ESP32 must stay powered while the e-stop is pressed (§8).
- **No arming or ownership protocol**, following ROS convention. Activating and deactivating the ros2_control hardware component or controllers is the host-side "enable": when they stop, the command stream stops and the timeout stops the wheels. Command sources are combined on the host (`twist_mux`). The firmware has exactly one command source (ROS). The console can e-stop but cannot drive (bench driving uses the DShot bring-up mode).

### 7.4 Wheel feedback

- After each `send()`, read `status()`, `erpm()` / `rpm()`, and `ageUs()` for each ESC.
  - `status() == DSHOT_RX_OK` means a valid reading. Note that eRPM `0` is a real "not spinning" value, not a lost frame.
  - The other status values (`NO_REPLY`, `FRAMING`, `BAD_GCR`, `BAD_CRC`) mark the sample invalid.
- Convert to wheel units:
  - motor RPM = eRPM × 2 / magnet count (the library's `rpm()`)
  - wheel ω (rad/s) = motor RPM / drive reduction × 2π / 60
- **Direction:** telemetry RPM is unsigned. When the command changes sign at speed, AM32 first slows the motor down and then reverses it. So the sign cannot simply follow the command. Track a per-wheel "actual direction" that flips only once measured RPM drops below a small threshold after a sign change.
- **Position:** integrate signed ω in the motor task at 1 kHz (double precision, radians). Because the position is accumulated on the MCU, lost `wheel_states` packets never lose distance. Invalid samples hold the last valid velocity for a short time, then count as zero and are flagged.
- `FeedbackStore` keeps the latest sample, a light low-pass-filtered velocity, the accumulated position, a validity flag (stale or no telemetry → invalid), and link health from the library's `stats()` / `lossPercent()`. History and plotting come from `ros2 bag` / PlotJuggler, so there is no on-device ring buffer.

### 7.5 micro-ROS node (`RosNode`)

- Owns the rclc support, node, executor, entities (§4.2) and preallocated message buffers. Created on connect and destroyed when the session is lost (§5.2).
- `wheel_commands` callback: decode → `WheelCommandHandler` → `IDriveController`.
- `wheel_states` is published from `IWheelFeedback` **right after each accepted wheel command**, so its rate follows the host's control rate with no firmware setting. If no command has arrived for `1 / state_idle_hz`, an idle timer publishes instead. The publish decision is a small core helper (`StatePublishPolicy`), tested on the host.
- A 1 Hz timer publishes `estop_state` and one diagnostics group. Groups:
  - per-ESC telemetry and link stats
  - drive state and timeout count
  - connection (interface, IP, reconnects)
  - motor loop timing (max period, missed deadlines)
- Parameter server callbacks validate changes (e.g. `motor_poles` only while stopped) and apply them through `IConfigStore`. `save_params` writes NVS.
- Timestamps use the agent-synced clock (`rmw_uros_epoch_nanos`).

### 7.6 Host bringup (`ros/`)

- **`colebot_description`:** xacro URDF with `base_link`, two continuous wheel joints (6 in wheels), and the chassis dimensions from `model/`. Its `ros2_control` tag selects:
  - `joint_state_topic_hardware_interface/JointStateTopicSystem` with:
    - `joint_commands_topic=/chassis/wheel_commands`
    - `joint_states_topic=/chassis/wheel_states`
    - **`trigger_joint_command_threshold=-1`**
  - or `mock_components/GenericSystem` when `use_mock_hardware:=true`.

  Each wheel joint has a `velocity` command interface and `position` + `velocity` state interfaces.
- **`colebot_bringup`:** launch and config for:
  - the micro-ROS agent (UDP 8888, discovery enabled; Docker `microros/micro-ros-agent:jazzy`, since there is no binary package)
  - `robot_state_publisher`
  - `ros2_control_node`, with `update_rate` from the launch argument `control_rate` (default **50**)
  - `joint_state_broadcaster`
  - `diff_drive_controller`:
    - `wheel_radius` 0.0762 m
    - `wheel_separation` measured from CAD
    - `position_feedback: true`
    - `publish_rate` = `control_rate`
    - `cmd_vel_timeout: 0.5`
    - velocity and acceleration limits
    - `enable_odom_tf: true` until the EKF takes over in sprint 4
  - `twist_mux`, with a **lock** on `/chassis/estop_state` at a priority above every input, so teleop, Nav2 and CLI commands are blocked on the host while the e-stop is latched. Inputs include `/cmd_vel/cli` for `ros2 colebot drive`
  - `wheel_velocity_controller` (`velocity_controllers/JointGroupVelocityController` on both wheel joints), loaded but **inactive**. `ros2 colebot tune` swaps it in for `diff_drive_controller` (§7.7)
  - `diagnostic_aggregator`
  - the Ethernet port on the computer configured with the matching static address

  The Jazzy `diff_drive_controller` takes `TwistStamped` on `cmd_vel`, so teleop and `twist_mux` are configured to publish stamped messages.
- **Control rate is one setting.** The `control_rate` launch argument sets `update_rate` and `publish_rate`. The firmware answers each command with a state, so it follows automatically. `cmd_timeout_ms` stays in milliseconds, so it doesn't depend on the rate.
- **Firmware reboot.** Wheel positions restart at 0, so `diff_drive_controller` odometry shows one bogus jump. No special handling for now. Restarting the host stack resets odometry if it matters.
- **Operation uses standard tools**, with `ros2 colebot` (§7.7) as a shortcut for the common cases:
  - `teleop_twist_keyboard`, `rqt_robot_steering` (driving)
  - `ros2 param` (tuning)
  - `ros2 service call /chassis/estop …`
  - `rqt_robot_monitor` (diagnostics)
  - `ros2 bag` + PlotJuggler (speed and odometry plots)

### 7.7 `ros2 colebot` command (`colebot_cli`)

A Python package in `ros/` that adds `ros2 colebot <verb>` to the `ros2` CLI, built the same way as `ros2 param` and `ros2 topic`. It runs on any machine on the ROS network (for example a laptop over WiFi) and only uses the §4 interface and the standard host nodes. It never talks to the firmware any other way.

**Packaging** (`ament_python`). `setup.py` registers:
```python
entry_points={
    'ros2cli.command': ['colebot = colebot_cli.command.colebot:ColebotCommand'],
    'ros2cli.extension_point': ['colebot_cli.verb = colebot_cli.verb:VerbExtension'],
    'colebot_cli.verb': [
        'status = colebot_cli.verb.status:StatusVerb',
        'drive = colebot_cli.verb.drive:DriveVerb',
        'estop = colebot_cli.verb.estop:EstopVerb',
        'clear-estop = colebot_cli.verb.clear_estop:ClearEstopVerb',
        'config = colebot_cli.verb.config:ConfigVerb',
        'tune = colebot_cli.verb.tune:TuneVerb',
    ],
}
```
- `ColebotCommand` follows `ros2param`'s `ParamCommand`: `add_subparsers_on_demand(...)`, then it dispatches to the verb.
- Verbs get a temporary node from `ros2cli.node.direct.DirectNode`. Tab completion comes from ros2cli.
- **The logic lives in plain modules, separate from ROS.** Status formatting, argument validation and step-response metrics are pure functions under `colebot_cli/core/`, so pytest can test them without a ROS graph. The verb classes stay thin.
- **Exit codes:** 0 on success, 1 if the request was refused (e.g. clear refused while the hardware e-stop is pressed), 2 if the chassis was not found within the timeout. This lets scripts and tests use them.

| Verb | What it does | Built on |
| ---- | ------------ | -------- |
| `status [--watch] [--timeout S]` | One summary: connection (interface, IP, reconnects), e-stop latch (state, first source, active trips), per-wheel speed / position / telemetry validity / ESC loss %, `wheel_states` rate, motor loop timing. `--watch` refreshes it in place | `/diagnostics_agg` (the aggregator keeps every group, so one sample is complete; falls back to `/diagnostics`), `/chassis/estop_state`, `/chassis/wheel_states` |
| `drive <linear m/s> <angular rad/s> [--for S] [--rate HZ]` | Publishes `TwistStamped` on `/cmd_vel/cli` at `--rate` (default 20) for `--for` seconds (default 1, max 10), then publishes zero. Ctrl-C also sends zero. Says so and exits 1 if the e-stop is latched | `twist_mux` input `/cmd_vel/cli` (priority below teleop) |
| `estop` / `clear-estop` | Calls the service and prints the firmware's reason when a clear is refused | `/chassis/estop`, `/chassis/clear_estop` |
| `config list` / `get <key>` / `set <key> <value> [--save]` / `save` | Firmware parameters with types checked before sending. `--save` calls `save_params` after a successful set | `/chassis` parameter services, `/chassis/save_params` |
| `tune step <rad/s> [--wheel left\|right\|both] [--for S] [--csv FILE]` | Speed-loop step test with the wheels off the ground. Asks for confirmation (`--yes` skips it). It then deactivates `diff_drive_controller`, activates `wheel_velocity_controller`, commands the step, records `wheel_states`, commands zero and switches back, even on Ctrl-C or error. It prints rise time (10–90 %), overshoot and steady-state error, and optionally saves the samples to CSV | `controller_manager` `switch_controller`, `/wheel_velocity_controller/commands` |

Notes on `tune`:
- **Samples arrive at the control rate** (20 ms at 50 Hz), which is enough to compare gains, not to study fast dynamics. Launch with `control_rate:=100` for tuning sessions if more detail helps.
- **The firmware's `max_wheel_accel` ramp still applies**, so a step becomes a ramp. Raise it temporarily while tuning, or the result mostly measures the ramp.
- The `twist_mux` lock doesn't cover this path because it bypasses `cmd_vel`. The firmware e-stop and command timeout still do.

**Order of work:** `status`, `estop` / `clear-estop` and `drive` come first, since they help with sprint 1 bring-up. `config` follows, then `tune` when the speed loop is tuned.

### 7.8 Service console (USB)

A minimal `esp_console` REPL on the DevKitC-1's native **USB** port (USB-Serial-JTAG, GPIO 12/13). The same cable also provides JTAG debugging. It covers only what ROS can't reach: bring-up, network setup, and recovery when there is no agent.

| Command                              | Purpose                                                       |
| ------------------------------------ | ------------------------------------------------------------- |
| `status`                             | Drive state, e-stop latch, wheel speeds, connection state / interface / IP, loop timing |
| `estop` / `clear-estop`              | E-stop latch (source `CONSOLE`)                               |
| `net eth <ip/mask> [gw]`             | Ethernet static address (WiFi always uses DHCP)               |
| `net wifi <ssid> <pass>`             | WiFi credentials                                              |
| `net agent <eth\|wifi> <ip:port> \| clear` | Saved agent address per interface (fallback when discovery fails) |
| `net reconnect`                      | Drop the session and re-enter `CONNECTING`                     |
| `dshot cmd <side> <cmd>`             | DShot special command (beacon, 3D mode, direction, save, EDT on/off), only while stopped |
| `dshot diag [side]`                  | `echoPulses()`, `status()`, link stats / loss %, armed state  |
| `config get\|set\|save <key> [value]`  | Same parameters as §4.2, for use without an agent             |

ESP_LOG output shares this console. When no host is attached, USB-Serial-JTAG writes can stall until they time out. That's fine for the `console` and `ros` tasks, and one more reason the motor task never logs.

### 7.9 Testing

The plan follows the usual ROS 2 practice:
- GoogleTest/GMock for C++
- pytest for Python
- `ament_lint` for style
- `launch_testing` for integration tests
- ros2_control mock hardware for the controller stack

| Layer | How it's tested | When |
| ----- | --------------- | ---- |
| Firmware `core` | Plain CMake + GoogleTest/GMock (`FetchContent`) in `colebot/test/host/`, with the `Mock*` / `Fake*` doubles. Not a colcon package: it must build without ROS or IDF, also on Windows | Every commit |
| Firmware platform (`RosNode`, network, `AlfredoEsc`) | Kept thin; covered by the hardware-in-the-loop tests | — |
| `colebot_cli` | pytest for the pure modules: status formatting from canned diagnostics, argument validation, step-response metrics on synthetic responses | Every commit |
| `ros/` packages | `colcon test` with `ament_lint_auto` + `ament_lint_common`; a `launch_testing` check that the URDF and controllers load with `use_mock_hardware:=true` | Every commit |
| Host stack without the robot (SIL) | `colebot_fake_chassis` implements the §4 contract: it follows commands with a first-order speed response, integrates positions, and implements the timeout, e-stop services and parameters. `launch_testing`: `cmd_vel` in → `diff_drive_controller` → topics → fake → odometry out. Also: the `wheel_states` rate follows `control_rate`, and drops to `state_idle_hz` when commands stop. Each `ros2 colebot` verb runs against the fake: `status` finds it, `drive` moves the odometry, an e-stop/clear round trip, `config set --save`, and `tune step` against the fake's first-order response, restoring `diff_drive_controller` afterwards | Every commit |
| Firmware + host on the robot (HIL) | `launch_testing` against the real agent and robot, automating the §7.10 criteria that don't need a person | Before merging firmware changes |

**`core` unit tests:**
- **`Motor`**, using `MockEsc` + `FakeClock`:
  - 3D mapping at the edges (0, ±min, ±1.0, NaN)
  - inversion
  - waiting for arming
  - direction tracking through a reversal
  - position integration across a reversal and across invalid samples
  - ESC restart sequence (`restart`, then waiting for `isArmed()`) with zero output throughout
  - invalid telemetry status
- **`PiController`:**
  - feedforward only
  - integral anti-windup at saturation
  - reset at zero command
  - minimum-speed deadband
- **`DriveController`**, using `MockMotor` + `FakeClock`:
  - ramp limit
  - command timeout (only wheel commands renew it; expiry ramps to zero)
  - session loss stops immediately
  - e-stop from several sources (first source + accumulated bitmask)
  - hardware e-stop with `FakeEstopInput`: latch on the first pressed sample; release needs 20 ms stable; release alone doesn't clear the latch; clear refused while pressed and during the ESC restart; both ESCs restarted when the trip ends
  - clear refused while a trip is active
  - after a clear, motion only after a zero command
- **`WheelCommandHandler`:**
  - name matching in any order
  - unknown or missing joints
  - NaN / inf rejected
  - clamping
- **`ConnectionManager`**, using `FakeNetworkInterface` × 2 + `FakeAgentLink` + `FakeClock`:
  - Ethernet wins when both have agents
  - WiFi is not accepted during `eth_grace_ms` while the Ethernet link is up
  - WiFi is used when Ethernet has no link or no agent
  - the other interface is stopped on connect
  - missed pings and Ethernet link-down → drive stopped, then `CONNECTING`
  - the interface is restarted on disconnect
  - no switch back while connected
- **`FeedbackStore`:**
  - filtering
  - staleness
  - unit conversion with non-default poles and reduction
- **`DiagnosticsBuilder`, `ConsoleCommands`:** output against mocked interfaces.
- **Pure helpers** (throttle mapping, conversions, filters) are tested directly.

**CI** (when the repo has one): ESP-IDF build in the `espressif/idf` container, the firmware host tests, and `ros-tooling/action-ros-ci` on Jazzy for `ros/`.

### 7.10 Sprint 1 acceptance criteria

- [ ] `teleop_twist_keyboard` (through `twist_mux` and `diff_drive_controller`) drives forward, reverse and turning. Measured wheel speeds track the commands within a reasonable tolerance after tuning.
- [ ] Odometry from `diff_drive_controller` matches a measured 2 m straight run and a 360° turn within a reasonable tolerance.
- [ ] Stopping the controller manager, killing the agent, or pulling the cable stops the wheels within `cmd_timeout_ms`.
- [ ] Connection:
  - booting with both links available connects over Ethernet, and the WiFi radio is off
  - pulling the cable fails over to WiFi without a reboot
  - booting without a cable connects over WiFi
  - after any failover the host stack resumes without restarting
- [ ] E-stop:
  - the `estop` service and the console `estop` stop the motors immediately
  - `estop_state` and diagnostics show the source
  - `clear_estop` doesn't restart motion until a zero command arrives
  - pressing the hardware e-stop cuts the motors, and `estop_state` / diagnostics show `HARDWARE`. `clear_estop` is refused while it is pressed. After release and a clear, the robot drives again without a reboot (ESCs restarted)
  - disconnecting the e-stop GPIO wire latches the e-stop
  - while latched, `twist_mux` blocks `cmd_vel` inputs
- [ ] `ros2 param set` changes drive parameters at runtime. `save_params` persists them across a reboot.
- [ ] `rqt_robot_monitor` shows ESC telemetry and link stats, drive state, connection state and motor loop timing.
- [ ] The motor task holds its 1 kHz period with no missed deadlines at the 50 Hz control rate over both Ethernet and WiFi, with diagnostics and the console busy. The same test at 100 Hz is recorded to show the headroom.
- [ ] Changing `control_rate` at launch changes both the controller rate and the firmware's `wheel_states` rate, with no other edits.
- [ ] Rebooting the ESP32 while the host runs: it reconnects and driving resumes without restarting the host stack.
- [ ] `core` host tests pass, covering the cases in §7.9. All hardware access goes through the interfaces in §3.3.
- [ ] `colcon test` passes for `ros/`, including the mock-hardware and `fake_chassis` launch tests.
- [ ] On the robot, `ros2 colebot status`, `drive`, `estop` / `clear-estop` and `config` work from a second machine on the ROS network. `tune step` produces a usable step response and always restores `diff_drive_controller`.

## 8. Hardware notes and constraints

- **RMT channels.** The ESP32-C6 RMT has **2 TX + 2 RX channels**. Two bidirectional DShot motors use all four. That leaves no RMT for anything else, including the dev board's addressable RGB LED, which normally uses RMT. A status LED would need another method (e.g. a plain GPIO LED, or SPI-driven WS2812, though GP-SPI2 is taken by the W5500).
- **One GPIO per ESC.** AlfredoDShot puts an RMT TX channel (open-drain) and an RX channel on the same pad, and the ESC replies on that same wire.
- **Signal wiring is out of scope** for this software (pull-up and series resistors are a hardware concern). The firmware only exposes what depends on it: the `dshot_push_pull` option and the `dshot diag` check (`echoPulses()` = 31 means the line is driven and released cleanly).
- **Push-pull caveat.** If push-pull is enabled, the line is released from the TX-done ISR. Flash writes (e.g. NVS saves) can delay that ISR and lose a reply, so `save_params` / `config save` are only allowed while stopped.
- **ESC firmware.** The ESCs run **AM32**, updated as needed: ≥ 2.21 is assumed so EDT current reads in 1 A steps. AM32 picks bidirectional mode by itself when the line idles high. 3D mode must be enabled.
- **AM32 configurator.** The library's `AM32_ConfiguratorLink` example needs USB-OTG (TinyUSB), which the C6 doesn't have. ESC configuration has to happen through a separate AM32 linker or by sending DShot commands from the console.
- **Pin choice.** Keep the DShot pins off the C6 strapping pins (GPIO 4, 5, 8, 9, 15) and the USB pins (GPIO 12/13). The DShot lines idle high, which would affect boot mode on strapping pins.
- **SPI.** The C6 has one general-purpose SPI controller (GP-SPI2), used by the W5500. SPI0/1 are reserved for flash.
- **UARTs.** UART0, UART1 and LP_UART are all spare now that the control link is network-based. On the DevKitC-1, UART0's default pins (GPIO 16/17) are wired to the on-board USB-UART bridge.
- **Hardware e-stop.** The e-stop cuts ESC power, not the ESP32's: the logic supply must stay up so the firmware keeps reporting state. The GPIO signal comes from an auxiliary contact on the e-stop. It must be 3.3 V-safe (isolated or level-shifted if it comes from the motor-power side), and wired so an open circuit reads as pressed (§7.3). Like the DShot signal wiring, the circuit itself is a hardware concern.
- **Pins to reserve now:** I²C SDA/SCL for the IMU and ADC pins for battery voltage/current sense (sprint 2+).
- **CPU.** The single core runs the 1 kHz motor task alongside lwIP, the WiFi driver and the W5500 RX task. WiFi and Ethernet are never both active once connected (§5.2), which keeps the load down.

## 9. Future sprints (high level)

### Sprint 2 — Battery & system monitoring
- Battery voltage and current via ADC (calibrated with `esp_adc_cali`), and/or via AM32 **Extended DShot Telemetry**:
  - Enable it with `DSHOT_CMD_EDT_ENABLE`. The library then exposes `temperatureC()`, `voltage()`, and `current()`.
  - Voltage and temperature update at ~5 Hz, current at ~25 Hz.
  - `current()` is in 1 A steps and assumes AM32 ≥ 2.21.
- `IBatteryMonitor` (ADC and/or EDT implementations), published as `sensor_msgs/BatteryState` on `/chassis/battery`. Low voltage is a warning in diagnostics, and an automatic e-stop trip at cutoff.
- More diagnostics: heap, task stacks, CPU load, reset reason, ESC temperatures.

### Sprint 3 — IMU
- `IImu` interface with an I²C (or SPI) driver implementation, and calibration of gyro bias and accelerometer offsets.
- Published as `sensor_msgs/Imu` on `/chassis/imu` at 100 Hz, stamped with the agent-synced clock. `imu_link` is added to the URDF.

### Sprint 4 — Localization (host)
- `robot_localization` EKF fusing `diff_drive_controller` odometry with the IMU. The EKF publishes `odom → base_link`, so `diff_drive_controller`'s `enable_odom_tf` is turned off.
- Nav2 readiness: `twist_mux` priorities for Nav2 vs teleop, footprint, velocity limits.
- Firmware changes are not expected. Goal-level behavior (rotate N degrees, drive N meters) is Nav2's or the host's job, not this controller's.

### Lyrical upgrade
- **Trigger:** `micro_ros_espidf_component` publishes a `lyrical` branch or a 26.x release. As of 2026-10-08 it has neither, though the rest of micro-ROS (agent, rmw, micro_ros_setup) has `lyrical` branches, and the host packages we use are released for Lyrical.
- **Steps:**
  - move the component submodule to the Lyrical release
  - move the host to Lyrical (Ubuntu 26.04)
  - rebuild the agent from its `lyrical` branch (or the Docker tag, once it exists)
  - rerun the full test suite, including HIL
- Firmware, agent and host always move together; mixed distros are not supported.
- Re-evaluate `joint_command_topic_hardware_interface` (`control_msgs/JointCommand`) then. It is on upstream `main` but not in the Jazzy release.

## 10. Decisions and open questions

### Decided

| Topic              | Decision                                                                                       |
| ------------------ | ---------------------------------------------------------------------------------------------- |
| DShot library      | AlfredoDShot v1.1, upstream submodule + IDF wrapper/shim                                       |
| ESC firmware       | AM32 (≥ 2.21, updated as needed), 3D mode                                                      |
| Signal wiring      | Out of scope for the software; firmware exposes `dshot_push_pull` and `dshot diag`            |
| Magnet count       | Default 14, configurable                                                                       |
| Drive reduction    | Default 1.0, configurable (set once the drivetrain is measured)                                |
| Dev board / console| ESP32-C6-DevKitC-1, service console on the native USB port (USB-Serial-JTAG)                   |
| Host interface     | micro-ROS over UDP; the custom `colebot-protocol`, UART link, BLE link and Python client are dropped |
| ROS distro         | Jazzy end to end (firmware, agent, host) until the ESP-IDF component supports Lyrical, then Lyrical end to end |
| Topology           | Host `ros2_control` (`diff_drive_controller` + `joint_state_topic_hardware_interface`); the MCU is a wheel-joint driver (rad/s in, position + velocity out) |
| Network            | W5500 SPI Ethernet + built-in WiFi. `CONNECTING` probes both (Ethernet first, with a grace period); the first agent wins and the other interface is stopped; no switch back while connected |
| Agent discovery    | Discovery on both interfaces, with a saved address as fallback; the wrong-agent risk is accepted for now |
| Safety             | Firmware command timeout (wheel commands only) + session-loss stop + e-stop latch (any source, first source + bitmask, clear refused during trips, zero command required after clear). No arming or ownership protocol (ROS convention) |
| Hardware e-stop    | Physical e-stop cuts ESC power; an auxiliary contact on a GPIO (fail-safe polarity) activates the `HARDWARE` trip. Release doesn't clear the latch; ESCs are restarted before a clear is accepted. `twist_mux` locks on `estop_state` |
| Console            | Minimal USB service console (network, status, e-stop, DShot, config); operation is via ROS tools |
| Testing            | GoogleTest/GMock host tests for firmware `core`; `colcon test` + `ament_lint` + `launch_testing` (mock hardware, `fake_chassis`) for `ros/`; HIL `launch_testing` on the robot |
| Host packages      | In this repo under `ros/`                                                                       |
| Odometry on firmware reboot | No special handling for now; odometry jumps once (§7.6) |
| Control rate       | 50 Hz to start; one launch argument (`control_rate`). The firmware replies to each command, so it needs no rate setting |
| Addressing         | Ethernet: static IP (placeholder `192.168.50.2/24`, computer `.1`). WiFi: DHCP |
| Project directory  | Keep `colebot/`                                                                                |
| Code structure     | Classes implement pure-abstract interfaces (where reasonable); constructor injection; `app_main` is the composition root; `core` is IDF- and micro-ROS-free |
| Naming             | Interfaces `I<Name>` in `i_<name>.h`; test doubles `Mock<Name>` (gMock, every interface) and `Fake<Name>` (in-memory, where useful) |
| License            | GPL-3.0-or-later (required by AlfredoDShot); SPDX header `GPL-3.0-or-later` on new source files. micro-ROS is Apache-2.0, which is compatible |
| CLI                | `ros2 colebot` (`colebot_cli`, ros2cli extension): `status`, `drive`, `estop` / `clear-estop`, `config`, `tune` (§7.7) |
| Plans              | [`docs/plans/alfredo-dshot.md`](docs/plans/alfredo-dshot.md)                                    |

### Open

None at the moment.

Values that will be measured later, such as drive reduction, wheel separation and the feedforward gains, are configurable with placeholder defaults.
