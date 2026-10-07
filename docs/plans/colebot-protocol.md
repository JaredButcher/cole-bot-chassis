# colebot-protocol — Sprint 1 Plan

The shared control protocol between the chassis controller and its hosts. It lives in its own repo so other projects can reuse it. This firmware consumes it as a git submodule.

> This document should move into the `colebot-protocol` repo as `docs/design.md` once that repo exists. `PLAN.md` §5.4 keeps only a summary.

| Item               | Value                                                                       |
| ------------------ | --------------------------------------------------------------------------- |
| Repo               | `github.com/JaredButcher/colebot-protocol`                                  |
| Mounted at         | `colebot/components/colebot_protocol`, relative URL `../colebot-protocol.git` (works over SSH and HTTPS). Tracks `main` until `v0.1.0`, then pinned to release tags |
| License            | **GPL-3.0-or-later**, so consumers must be GPL-3.0-compatible             |
| Message encoding   | Protocol Buffers (proto3)                                                   |
| MCU runtime        | nanopb **0.4.9.2**, static allocation only                                  |
| Host runtime       | Python `protobuf` (C++ hosts use the same nanopb build)                     |
| Framing            | COBS + CRC-16/CCITT-FALSE, `0x00` delimiter                                 |
| Languages          | C++17 (framing + nanopb glue), Python ≥ 3.10                                |
| Consumers (sprint 1) | This firmware (ESP-IDF component), Python host tools/tests                |

## 1. Goals and non-goals

**Goals**
- One schema (`colebot.proto`) is the single source of truth for every message.
- The same framing and messages work byte for byte in C++ and Python, verified by shared golden vectors.
- Builds three ways with no changes: as an ESP-IDF component, as a plain CMake library (Linux/Windows/macOS), and as a pip-installable Python package.
- No heap, no exceptions, no RTTI on the C++ side. Buffers have fixed sizes known at compile time.
- Consumers need no `protoc` or nanopb generator; generated code is committed.

**Non-goals (sprint 1)**
- Transport code (UART/BLE drivers). That belongs to each consumer. The firmware's `IControlTransport` lives in the firmware repo.
- Request routing, retries, or a session layer beyond `seq` + `Ack`/`Nack`.
- Encryption or authentication.

## 2. Repository layout

```
colebot-protocol/
├── proto/
│   ├── colebot.proto                 # Envelope + all messages (§4)
│   └── colebot.options               # nanopb max_size / max_count, so every field is static
├── generated/
│   └── nanopb/                       # colebot.pb.c / colebot.pb.h — committed
├── third_party/
│   └── nanopb/                       # vendored runtime: pb.h, pb_common.*, pb_encode.*, pb_decode.*, LICENSE.txt
├── cpp/
│   ├── include/colebot_protocol/
│   │   ├── protocol.h                # kProtocolVersion, size constants, encodeFrame(), FrameDecoder
│   │   ├── cobs.h                    # cobsEncode / cobsDecode (exposed for tests)
│   │   └── crc16.h                   # crc16CcittFalse (exposed for tests)
│   └── src/                          # cobs.cpp, crc16.cpp, protocol.cpp
├── python/
│   ├── pyproject.toml                # package "colebot-protocol"
│   └── colebot_protocol/
│       ├── __init__.py
│       ├── framing.py                # cobs, crc16, encode_frame, FrameDecoder
│       ├── client.py                 # Client: seq tracking, request/response, setpoint hold, streaming
│       ├── tool.py                   # `colebot-ctl` command-line tool (§7)
│       └── _generated/colebot_pb2.py # committed
├── tests/
│   ├── cpp/                          # GoogleTest
│   ├── python/                       # pytest
│   └── vectors/vectors.txt           # golden frames shared by both suites (§8)
├── scripts/
│   ├── generate.py                   # regenerate generated/ + _generated/ from proto/
│   └── make_vectors.py               # regenerate tests/vectors/ from Python definitions
├── requirements-dev.txt              # nanopb==0.4.9.2 (generator + protoc via grpcio-tools), pytest
├── CMakeLists.txt                    # dual-mode: ESP-IDF component or plain CMake (§6)
├── .github/workflows/ci.yml
├── CHANGELOG.md
├── LICENSE
└── README.md
```

**Vendor the nanopb runtime instead of nesting a submodule (decided).**
- The nanopb runtime is only 4 `.c` files plus headers, under the zlib license.
- Vendoring them pinned to 0.4.9.2 means consumers don't need `git submodule update --recursive`.
- The generator is a dev-only dependency, installed from PyPI (`nanopb==0.4.9.2`) through `requirements-dev.txt`.
- The vendored runtime and the generator must stay on the same version, because generated code checks `PB_PROTO_HEADER_VERSION`. `scripts/generate.py` asserts that.

## 3. Wire format

### 3.1 Frame

```
frame   = COBS( payload ‖ crc16_le ) 0x00
payload = protobuf-encoded colebot.Envelope
crc16   = CRC-16/CCITT-FALSE over payload (poly 0x1021, init 0xFFFF, no reflection, xorout 0x0000)
```

- **COBS** removes every `0x00` from the frame body, so `0x00` always marks a frame boundary. A receiver joining mid-stream resyncs at the next `0x00`. Overhead is at most 1 byte per 254 bytes plus 1.
- **CRC** is appended little-endian *before* COBS encoding, so it is protected by the framing too. Check value for `"123456789"` is `0x29B1`, which is the first unit test in both languages.
- **Size limits.** Max payload is nanopb's generated `colebot_Envelope_size`. `kMaxFrameSize = cobs_max(kMaxEnvelopeSize + 2) + 1`. The expected size is under 128 bytes. Receivers drop any frame longer than this and count an overflow.

### 3.2 Envelope

Every frame carries exactly one `Envelope`:

| Field              | Type     | Notes                                                                           |
| ------------------ | -------- | ------------------------------------------------------------------------------- |
| `protocol_version` | uint32   | `kProtocolVersion` (starts at 1). Receivers `Nack(VERSION_MISMATCH)` on a different value. |
| `seq`              | uint32   | Sender's counter, incremented per frame. Each direction has its own counter.     |
| *(3–9)*            | reserved | For future header fields                                                       |
| `payload`          | oneof    | The message (§4): requests 10–63, responses and streams 64+                    |

### 3.3 Request / response rules

- Host → robot commands get exactly one reply: a specific response (`Pong`, `Version`, `State`) or an `Ack`/`Nack`. The reply carries `request_seq` = the request's `seq`.
- This includes high-rate setpoint commands (decided). At 20 Hz the extra `Ack` traffic is ~20 small frames/s. In exchange, the host always knows whether each setpoint was accepted or why it was rejected (`NOT_ARMED`, `ESTOPPED`, `NOT_ACTIVE_SOURCE`, …).
- Robot → host stream messages (`WheelFeedback`) are not acknowledged.
- The robot never initiates requests in sprint 1.
- Hosts should treat no reply within ~100 ms as a lost frame. Commands are idempotent setpoints, so resending is safe.

## 4. Schema v1 (draft)

```proto
syntax = "proto3";
package colebot;   // nanopb types become colebot_Envelope, colebot_SetWheelCommand, ...

enum ErrorCode {
  ERROR_CODE_UNSPECIFIED       = 0;
  ERROR_CODE_UNKNOWN_MESSAGE   = 1;
  ERROR_CODE_INVALID_ARGUMENT  = 2;  // NaN, out of range
  ERROR_CODE_NOT_ARMED         = 3;
  ERROR_CODE_NOT_ACTIVE_SOURCE = 4;  // robot is armed by (claimed by) a different control source
  ERROR_CODE_ESTOPPED          = 5;
  ERROR_CODE_VERSION_MISMATCH  = 6;
  ERROR_CODE_BUSY              = 7;  // e.g. ESCs still in their arming window
  ERROR_CODE_CONDITION_ACTIVE  = 8;  // ClearEStop refused: an automatic latch condition is still present
}

enum ArmState {
  ARM_STATE_UNSPECIFIED = 0;
  ARM_STATE_DISARMED    = 1;
  ARM_STATE_ARMED       = 2;
  ARM_STATE_ESTOPPED    = 3;  // latched until ClearEStop
}

// Why the robot latched into ARM_STATE_ESTOPPED. Values double as bit positions in
// State.latch_sources / State.active_conditions (bit = 1 << value), so they must stay < 32.
//   1–7   command sources (an EStop request from an interface)
//   8–31  automatic causes (conditions detected by the robot)
enum LatchSource {
  LATCH_SOURCE_UNSPECIFIED  = 0;   // never set as a bit
  LATCH_SOURCE_CONTROL_LINK = 1;   // EStop over the UART control link
  LATCH_SOURCE_CLI          = 2;   // `estop` on the console
  LATCH_SOURCE_BLE          = 3;   // sprint 3
  LATCH_SOURCE_LOW_BATTERY  = 8;   // sprint 2
  LATCH_SOURCE_ESC_FAULT    = 9;   // ESC lost arming / stopped responding while armed (trip rules TBD)
  LATCH_SOURCE_HW_INPUT     = 10;  // future physical e-stop input
  LATCH_SOURCE_INTERNAL     = 11;  // firmware fault (e.g. motor task overrun)
}

// Why the most recent claim ended (armed -> not armed).
enum ReleaseReason {
  RELEASE_REASON_UNSPECIFIED    = 0;   // no claim has ended since boot
  RELEASE_REASON_DISARM_COMMAND = 1;   // a source sent Disarm (see State.last_release_by)
  RELEASE_REASON_LEASE_EXPIRED  = 2;   // the owner stopped sending motion commands
  RELEASE_REASON_ESTOP          = 3;   // the e-stop latch was entered
}

enum StateTrigger {
  STATE_TRIGGER_UNSPECIFIED = 0;
  STATE_TRIGGER_REPLY       = 1;   // answer to GetState (request_seq set)
  STATE_TRIGGER_CHANGE      = 2;   // pushed immediately on a state change
  STATE_TRIGGER_PERIODIC    = 3;   // stream tick at ConfigureStream.state_hz
}

// Which interface currently owns (has armed) the robot. A source is identified by the
// link a request arrives on, not by a field in the message.
enum ControlSource {
  CONTROL_SOURCE_NONE         = 0;   // disarmed / e-stopped: nobody owns the robot
  CONTROL_SOURCE_CONTROL_LINK = 1;   // UART control link
  CONTROL_SOURCE_CLI          = 2;   // console
  // CONTROL_SOURCE_BLE       = 3;   // sprint 3
}

// ---- requests (host -> robot) ----
message Ping            { uint32 token = 1; }
message GetVersion      {}
message Arm             {}                                   // claims control for the sending source and starts its lease (see notes)
message Disarm          {}                                   // accepted from any source; releases the claim, output ramps down
message EStop           {}
message ClearEStop      {}
message SetWheelCommand { float left = 1; float right = 2; }  // normalized -1..1 per wheel; renews the command lease (see notes)
message SetArcadeCommand{ float forward = 1; float turn = 2; } // normalized -1..1, mixed on the robot
message GetState        {}
message ConfigureStream {                                     // only fields present are changed
  optional uint32 wheel_feedback_hz = 1;                      // 0 = off (boot default); clamped to a max (e.g. 200)
  optional uint32 state_hz          = 2;                      // 0 = off; boot default 2; clamped to a max (e.g. 20)
}

// ---- responses / streams (robot -> host) ----
message Ack     { uint32 request_seq = 1; }
message Nack    { uint32 request_seq = 1; ErrorCode code = 2; }
message Pong    { uint32 request_seq = 1; uint32 token = 2; uint64 uptime_us = 3; }
message Version { uint32 request_seq = 1; uint32 protocol_version = 2;
                  string firmware_version = 3; string git_hash = 4; }   // max_size 32 / 16
message State {
  uint32        request_seq        = 1;   // set only when trigger == REPLY
  StateTrigger  trigger            = 2;
  uint32        state_change_count = 3;   // +1 on every CHANGE-worthy transition (see notes)
  ArmState      arm_state          = 4;
  ControlSource active_source      = 5;
  float         left_setpoint      = 6;   // after arbitration
  float         right_setpoint     = 7;
  float         left_output        = 8;   // after ramp
  float         right_output       = 9;
  LatchSource   first_latch_source = 10;  // what tripped the latch first; UNSPECIFIED unless ESTOPPED
  uint32        latch_sources      = 11;  // bitmask: every source that tripped since the latch was set
  uint64        latched_at_us      = 12;  // uptime when the latch was first set; 0 unless ESTOPPED
  uint32        active_conditions  = 13;  // bitmask: automatic sources (8–31) whose condition is present now
  ReleaseReason last_release_reason = 14; // why the last claim ended
  ControlSource last_release_by     = 15; // who sent the Disarm (DISARM_COMMAND only), else NONE
}
message WheelSample   { float motor_rpm = 1;   // signed, direction-tracked
                        float speed_mps = 2;   // signed
                        bool valid = 3; uint32 age_us = 4; float loss_percent = 5; }
message WheelFeedback { uint64 timestamp_us = 1; WheelSample left = 2; WheelSample right = 3; }

message Envelope {
  uint32 protocol_version = 1;
  uint32 seq = 2;
  reserved 3 to 9;
  oneof payload {
    Ping ping = 10;  GetVersion get_version = 11;  Arm arm = 12;  Disarm disarm = 13;
    EStop estop = 14;  ClearEStop clear_estop = 15;
    SetWheelCommand set_wheel_command = 16;  SetArcadeCommand set_arcade_command = 17;
    GetState get_state = 18;  ConfigureStream configure_stream = 19;
    // 20–63 future requests
    Ack ack = 64;  Nack nack = 65;  Pong pong = 66;  Version version = 67;  State state = 68;
    WheelFeedback wheel_feedback = 69;
    // 70+ future responses / streams
  }
}
```

Design notes:
- **Normalized commands only in sprint 1.** Control is open-loop throttle, so commands are `-1..1`. `SetArcadeCommand` is deliberately *not* named "Twist", which implies physical units. When closed-loop control arrives, add a separate `SetVelocityCommand { linear_mps, angular_radps }` without changing the existing messages.
- **Control ownership: Arm claims, Disarm releases (decided).**
  - **Invariant: armed ⇔ claimed.** `arm_state == ARMED` exactly when `active_source != NONE`. Every transition changes both together, atomically, and pushes one `State` `CHANGE`.
  - **Sources.** A request's source is the link it arrives on: UART = `CONTROL_LINK`, console = `CLI`, later BLE. Messages carry no source field.
  - **`Arm`** claims the robot for the sending source, which becomes `active_source`:
    - from a disarmed robot: `Ack`. The robot is armed and owned by the sender, and **a lease starts**. The owner must send its first motion command (a zero setpoint is fine) before the lease runs out, or the robot disarms again.
    - from the source that already owns the armed robot: `Ack`, with no change. It does **not** renew the lease.
    - while armed by a **different** source: `Nack(NOT_ACTIVE_SOURCE)`. Taking over is always two explicit steps: `Disarm`, then `Arm`. Ownership can never change silently while the robot is moving.
    - while e-stopped: `Nack(ESTOPPED)`
    - while the ESCs are still in their arming window: `Nack(BUSY)`
  - **`Disarm`** is accepted from **any** source, whoever owns the robot. It immediately sets `arm_state = DISARMED` and `active_source = NONE`; the output then ramps down to zero. If a source `Arm`s during the ramp-down, output continues from wherever the ramp has got to.
  - **Motion commands** (`SetWheelCommand`, `SetArcadeCommand`, later `SetVelocityCommand`) are acted on and acknowledged only from the active source:
    - disarmed: `Nack(NOT_ARMED)`
    - armed by another source: `Nack(NOT_ACTIVE_SOURCE)`
    - e-stopped: `Nack(ESTOPPED)`

    A rejected command never renews the lease.
  - **Lease expiry disarms and releases the claim (decided).** It behaves exactly like a `Disarm`: `DISARMED`, `active_source = NONE`, output ramps down. The difference is `last_release_reason = LEASE_EXPIRED`, so the former owner can tell its own lease lapsed rather than someone disarming it. A robot left alone therefore never stays armed. To stay armed while stationary, the owner keeps streaming zero setpoints.
  - **E-stop releases the claim.** Entering the latch sets `active_source = NONE` and `last_release_reason = ESTOP`. After `ClearEStop` the robot is `DISARMED` with no owner.
  - Every change of `active_source` is a `State` `CHANGE` event, so the previous owner is told immediately when it loses control.
  - **Any source, regardless of ownership:** `EStop`, `ClearEStop`, `Disarm`, and the read-only/diagnostic requests (`Ping`, `GetVersion`, `GetState`, `ConfigureStream`). Stream settings are per link.
- **Motion commands and the failsafe: the command lease.**
  - `Arm` and every accepted motion command grant a *lease*: a deadline. If no newer motion command replaces it first, the robot **disarms**: the claim is released and output ramps to zero (see ownership above).
  - The failsafe is simply "the lease expired". There is no separate heartbeat message.
  - Nothing else renews a lease: not `Ping`, not `GetState`, not any other traffic. The robot only keeps moving while whatever produces the commands keeps producing them, so a crashed host control loop can't be masked by a live ping or diagnostics thread.
  - **Setpoint commands** (`SetWheelCommand`, `SetArcadeCommand`, and a future `SetVelocityCommand`) get a short lease equal to the failsafe timeout (default 250 ms). Hosts resend the current setpoint at a steady rate, even when it hasn't changed. 20 Hz is recommended, which tolerates about 4 lost frames in a row.
  - To stay armed while stationary, the owner streams zero setpoints. Stopping the stream is a valid way to finish: the lease expires and the robot disarms by itself.
  - A future **goal command** (e.g. `RotateDegrees`, `MoveMeters`) would carry its own *bounded* lease: a required `timeout_ms` plus speed limits, ending early on completion. Any newer motion command, `Disarm`, or `EStop` cancels it. Sprint 1 has no goal commands. Whether they belong on this controller at all is deferred; see `PLAN.md` §8.
- **E-stop latch.**
  - Entering: `ARM_STATE_ESTOPPED` is entered by an `EStop` request from any interface, or by an automatic trip (from sprint 2). `EStop` is accepted from any source in any state and is never refused. Output goes to zero immediately, with no ramp.
  - **Multiple sources.** The first trip sets `first_latch_source` and `latched_at_us`. Every trip, including later ones while already latched, ORs its bit into `latch_sources`. So the host sees both what happened first and everything that has happened since. A repeat `EStop` from a source that is already in `latch_sources` changes nothing.
  - **Latched vs. active.** `latch_sources` is history: it stays set until cleared. `active_conditions` is the present: an automatic cause such as low battery sets its bit while the condition exists and clears it when the condition goes away. Command sources (1–7) never appear in `active_conditions`.
  - **Clearing (decided).** `ClearEStop` is accepted from **any** source; it doesn't matter which source tripped the latch. It is refused with `Nack(CONDITION_ACTIVE)` while any automatic trip is still active (`active_conditions != 0`). Once those conditions are gone, any source can clear the latch. A successful clear resets `first_latch_source`, `latch_sources`, and `latched_at_us`, and moves to `DISARMED`. An explicit `Arm` is then needed before driving.
- **Disarm vs. EStop output (decided).** `Disarm` ramps the output down to zero using the normal slew limit, then blocks further output. `EStop` (and automatic trips) zero the output immediately, with no ramp.
  - While latched, `Arm` and motion commands get `Nack(ESTOPPED)`.
- **State notification: push on change, plus a periodic backstop.**
  - **On change (`CHANGE`):** the robot sends `State` immediately whenever `arm_state`, `active_source`, `first_latch_source`, `latch_sources`, or `active_conditions` changes, and increments `state_change_count` each time. Setpoint and output values alone do **not** trigger a `CHANGE` because they move constantly.
  - **Periodic (`PERIODIC`):** sent at `state_hz` (boot default 2 Hz). A lost `CHANGE` frame is corrected by the next periodic one, and the stream doubles as a robot-alive signal.
  - **Missed-transition detection:** if `state_change_count` jumps by more than 1 between `State` frames, the host missed a transition, e.g. an e-stop that was tripped and cleared during frame loss.
  - **Host rules:** treat each `State` as the source of truth. While driving, the `Nack(ESTOPPED)` on the next setpoint is the fastest signal (≤ 50 ms at 20 Hz). Treat no `State` for more than ~3 periods as a lost link.
  - All three are robot → host messages and are not acknowledged, so the robot still never initiates requests.
- **Requests vs. responses are split by field-number range**, which makes sniffing and debugging easy.
- **Evolution rules:**
  - Never renumber or reuse fields; `reserved` replaces anything removed.
  - New fields are additive.
  - `protocol_version` changes only for semantic breaks.

## 5. C++ API (sprint 1)

```cpp
namespace colebot::protocol {

inline constexpr uint32_t kProtocolVersion = 1;
inline constexpr size_t   kMaxEnvelopeSize = colebot_Envelope_size;
inline constexpr size_t   kMaxFrameSize    = /* cobs_max(kMaxEnvelopeSize + 2) + 1 */;

// Encodes env into a complete frame (including the trailing 0x00).
// Returns bytes written, or 0 if encoding fails or cap is too small.
size_t encodeFrame(const colebot_Envelope& env, uint8_t* out, size_t cap);

struct DecoderStats { uint32_t frames, overflow, cobs_errors, crc_errors, decode_errors; };

class FrameDecoder {
 public:
  enum class Result : uint8_t { kNeedMore, kFrame, kError };
  // Feed one byte. On kFrame, `out` holds the decoded envelope.
  Result push(uint8_t byte, colebot_Envelope& out);
  const DecoderStats& stats() const;
  void reset();
 private:
  uint8_t buf_[kMaxFrameSize];
  size_t  len_ = 0;
  bool    discarding_ = false;   // after overflow, drop bytes until next 0x00
  DecoderStats stats_{};
};

// Exposed for testing / reuse
size_t   cobsEncode(const uint8_t* in, size_t len, uint8_t* out, size_t cap);
size_t   cobsDecode(const uint8_t* in, size_t len, uint8_t* out, size_t cap);
uint16_t crc16CcittFalse(const uint8_t* data, size_t len);

}  // namespace colebot::protocol
```

- Byte-at-a-time `push()` keeps the decoder trivially correct across partial reads. Throughput isn't a concern at UART rates.
- The firmware's `ControlLink` owns a `FrameDecoder` and calls `encodeFrame` into a stack buffer. The library does no I/O.
- nanopb build flags: `PB_ENABLE_MALLOC` **off**. `PB_NO_ERRMSG` stays off on the host, with an option to turn it on for the MCU to save flash.

## 6. Build integration

A single top-level `CMakeLists.txt` serves both modes:

```cmake
if(ESP_PLATFORM)                       # set by ESP-IDF's build system
  idf_component_register(
    SRCS  cpp/src/cobs.cpp cpp/src/crc16.cpp cpp/src/protocol.cpp
          generated/nanopb/colebot.pb.c
          third_party/nanopb/pb_common.c third_party/nanopb/pb_encode.c third_party/nanopb/pb_decode.c
    INCLUDE_DIRS cpp/include generated/nanopb third_party/nanopb)
else()
  cmake_minimum_required(VERSION 3.16)
  project(colebot_protocol LANGUAGES C CXX)
  add_library(colebot_protocol ...same sources...)
  target_include_directories(colebot_protocol PUBLIC ...)
  target_compile_features(colebot_protocol PUBLIC cxx_std_17)
  if(PROJECT_IS_TOP_LEVEL) # build tests only when developing this repo
    enable_testing(); add_subdirectory(tests/cpp)
  endif()
endif()
```

- **Firmware:** the submodule sits in `components/`, so ESP-IDF picks it up automatically. `core` lists `colebot_protocol` in `REQUIRES`.
- **Firmware host tests:** `colebot/test/host/CMakeLists.txt` does `add_subdirectory(../../components/colebot_protocol ...)` and links `colebot_protocol`. Because the protocol project isn't top level there, its own tests are skipped.
- **Python:** `pip install ./python` (or `pip install "git+https://...#subdirectory=python"`). Runtime dependency is `protobuf>=<gencode version>`. `pyserial` is an optional extra (`colebot-protocol[serial]`) used by `Client` and `colebot-ctl`.

## 7. Python package

- `framing.py`: `cobs_encode` / `cobs_decode`, `crc16_ccitt_false`, `encode_frame(env) -> bytes`, and `FrameDecoder.feed(data: bytes) -> list[Envelope]` with the same stats as C++. Implemented here rather than with the `cobs` PyPI package, so the two implementations are verified to match each other.
- `client.py`: `Client(stream)` wraps any object with `read`/`write`, e.g. a `serial.Serial`.
  - `request(msg, timeout=0.1)`: assigns `seq`, waits for the matching reply, retries once.
  - Typed helpers: `ping()`, `arm()`, `disarm()`, `estop()`, `set_wheels(l, r)`, ...
  - `session(rate_hz=20)`: a context manager that sends `Arm`, then keeps resending the current setpoint (zero at first) from a background thread, and sends `Disarm` on exit. `session.set_wheels(l, r)` / `set_arcade(f, t)` change what is streamed. Because a lone `Arm` disarms after 250 ms, this is the normal way to drive from Python.
  - `on_feedback(callback)` for streamed `WheelFeedback`.
- `tool.py` provides the `colebot-ctl` entry point, which sprint 1 uses to drive the robot over UART and pass the acceptance tests:
  ```
  colebot-ctl --port /dev/ttyUSB0 ping | version | state | disarm | estop | clear-estop
  colebot-ctl --port COM5 drive 0.2 0.2 --for 2s      # Arm, stream the setpoint at 20 Hz, then Disarm (Ctrl+C disarms early)
  colebot-ctl --port COM5 stream --hz 50 [--csv out.csv]
  ```

## 8. Testing

| Suite               | Runner     | Cases                                                                                     |
| ------------------- | ---------- | ----------------------------------------------------------------------------------------- |
| CRC / COBS          | both       | Standard check value (`0x29B1`); COBS vectors from the COBS paper (empty, all-zero, 254/255-byte runs); encode/decode round-trip on random data |
| Frame decoder       | both       | Single frame; frames split across arbitrary byte boundaries; back-to-back frames; garbage before and between frames; bad CRC; COBS error; oversize frame then recovery; empty frame (`0x00 0x00`) ignored |
| Messages            | both       | Round-trip every payload type with non-default values, including `latch_sources` bitmasks with several bits set and `ConfigureStream` with only one `optional` field present; unknown oneof field number (newer peer) → decode OK with `which_payload == 0`, which the firmware answers with `Nack(UNKNOWN_MESSAGE)`; NaN/inf floats survive the round trip |
| **Golden vectors**  | both       | `tests/vectors/vectors.txt` lines are `<name> <frame-hex>`, produced by `scripts/make_vectors.py` from Python definitions. **Python** asserts encode == hex and decode == expected message. **C++** asserts decode succeeds and re-encode == the exact same bytes. Together these prove both directions match byte for byte |
| Fuzz-lite           | C++        | Feed 1 MB of random bytes into `FrameDecoder`: no crash, no out-of-bounds read or write (run under ASan/UBSan in CI) |
| Generated code      | CI         | `scripts/generate.py` then `git diff --exit-code` — committed code matches the schema      |

Test doubles here follow the firmware's naming rule (`Mock<Name>` / `Fake<Name>`). For example, `FakeSerial` in Python tests is an in-memory stream for `Client`.

## 9. CI (GitHub Actions)

- `generate-check`: install `requirements-dev.txt`, run `scripts/generate.py`, then `git diff --exit-code`.
- `cpp`: CMake + GoogleTest (FetchContent) on `ubuntu-latest` (GCC, with ASan/UBSan) and `windows-latest` (MSVC).
- `python`: pytest on Python 3.10 and 3.13.
- `esp-idf` (optional, later): build a minimal IDF project that `REQUIRES colebot_protocol` for `esp32c6` in the `espressif/idf:v5.5.x` container.

## 10. Versioning and release

- Semver tags, starting at `v0.1.0` for sprint 1. Stay on `0.x` while the schema is still settling.
- `CHANGELOG.md` notes schema changes explicitly (added messages and fields).
- The package version and `kProtocolVersion` are independent: package versions change often, `kProtocolVersion` only on breaking semantics.
- The firmware pins the submodule to a tag. Bumping it is a normal firmware PR.

## 11. Work breakdown (sprint 1)

| #  | Task                                                                                         | Done when                                                         |
| -- | -------------------------------------------------------------------------------------------- | ----------------------------------------------------------------- |
| P1 | Repo exists (`main` @ `1bf26b8`: GPL-3.0 `LICENSE`, C++ `.gitignore`, stub README) and is added as a submodule. Remaining work: README with the GPL-3.0-or-later notice, SPDX header convention, Python entries in `.gitignore` (`__pycache__/`, `.venv/`, `*.egg-info/`, `dist/`), layout skeleton, CI skeleton | CI runs green on an empty test |
| P2 | COBS + CRC + `FrameDecoder` in C++ and Python, with tests                                    | Framing tests pass in both languages                              |
| P3 | `colebot.proto` + `.options`, vendor nanopb 0.4.9.2, `scripts/generate.py`, commit generated code | `generate-check` passes; nanopb sizes are static (no callbacks)  |
| P4 | `encodeFrame` + envelope decode, message round-trip tests                                    | Message tests pass in both languages                              |
| P5 | Golden vectors (`make_vectors.py`, both suites)                                              | Cross-language vector tests pass                                  |
| P6 | Dual-mode `CMakeLists.txt`; add as submodule in the firmware; firmware builds for `esp32c6`  | `idf.py build` succeeds with `core` linking `colebot_protocol`     |
| P7 | Python `Client` + `colebot-ctl`                                                              | Can ping, arm, drive, and stream against the firmware             |
| P8 | Tag `v0.1.0`, pin in the firmware                                                            | Firmware submodule points at the tag                              |

P1–P5 don't need hardware and can start right away, in parallel with firmware sprint 0. P7's end-to-end check needs firmware `ControlLink` to be working.

## 12. Decisions needed

1. ~~License~~ → **GPL-3.0-or-later** (decided). The `LICENSE` file is the standard GPL-3.0 text; "or later" is declared in the README and in `SPDX-License-Identifier: GPL-3.0-or-later` headers. Vendored nanopb stays zlib-licensed, which is GPL-compatible.
2. ~~nanopb packaging~~ → **vendored** (decided). The 0.4.9.2 runtime lives in `third_party/nanopb/` with its zlib `LICENSE.txt`. The generator is a dev-only pip dependency pinned to the same version.
3. ~~E-stop semantics~~ → **decided:**
   - The latch is shared by command e-stops and automatic trips.
   - The latch records the first source plus an accumulated `latch_sources` bitmask; `active_conditions` shows which automatic trips are still present.
   - `ClearEStop` is accepted from any source, but refused with `Nack(CONDITION_ACTIVE)` while an automatic trip is active.
   - After a clear, an explicit `Arm` is required.
4. ~~Command units~~ → **decided:** `SetWheelCommand` and `SetArcadeCommand` are normalized (−1..1). A velocity command in real units (m/s, rad/s) comes later with the closed-loop features, as a new message.
5. ~~Who may send `ClearEStop`~~ → **any source** (decided); see 3.
6. ~~`Disarm` behaviour~~ → **ramps down** (decided). `EStop` and automatic trips stop immediately.
