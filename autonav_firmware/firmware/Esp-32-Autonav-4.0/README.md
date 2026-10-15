# Esp-32-Autonav-4.0

ESP32 firmware for AutoNav's peripheral controller board. This is **not** a
motor controller and does **not** run micro-ROS — motor control now happens
entirely on the ROS2 side over RS485/Modbus (see `autonav_firmware`'s
`AutonavInterface`/`MotorController`). This board only does three things:

1. **Device identity** — a persisted "serial ID" the ROS-side bridge node
   queries/confirms on connect.
2. **LED control** — drives a 45-pixel WS2812 strip with switchable,
   non-blocking animation patterns.
3. **Emergency feedback** — reads a physical e-stop button and reports it to
   ROS, and receives emergency state from ROS to drive an alert LED pattern.

It talks to the robot's onboard computer over a single UART link: plain
serial at **115200 baud**, framed with a small custom binary protocol (no
micro-ROS, no newline/CSV parsing). The ROS-side counterpart
(`esp32_bridge_node`) lives in the `autonav_firmware` ROS2 package — see
`../../README.md` in that package for the ROS-side half of this link.

## Hardware

| Signal | Pin (`include/board_config.h`) | Notes |
|---|---|---|
| LED data | `LED_PIN` = 13 | WS2812, `NUM_LEDS` = 45, GRB order |
| E-stop input | `ESTOP_PIN` = 27 | Active-low, `INPUT_PULLUP`, 30 ms debounce |
| UART | ESP32's default `Serial` (USB/UART0) | 115200 baud, 8N1 |

All of the above are `constexpr` in `include/board_config.h` — change them
there, not in the library code, if the wiring differs on a given board
revision.

## Project layout

PlatformIO convention: each module is its own private library under
`lib/<Name>/{include,src}` (auto-discovered by PlatformIO's dependency
finder), so each concern is a real, independently-compiled OOP unit instead
of one flat `src/` file.

```
Esp-32-Autonav-4.0/
├── platformio.ini
├── include/
│   └── board_config.h          # pin map, timing constants, default device id
├── lib/
│   ├── AutonavProtocol/        # wire protocol: Frame, CommandId, CRC8, encoder/decoder
│   ├── AutonavIdentity/        # DeviceIdentity — NVS-backed device id
│   ├── AutonavEstop/           # EstopMonitor — debounced e-stop GPIO
│   └── AutonavLed/             # LedController + LedPattern hierarchy
│       └── patterns/           # OffPattern, FadeWaitPattern, BreathePattern,
│                                # CometPattern, BlinkPattern, EmergencyAlertPattern
├── src/
│   └── main.cpp                # wires everything together; setup()/loop()
└── test/
    └── test_protocol/          # PlatformIO/Unity tests for the frame codec
```

### `AutonavLed` design

`LedPattern` is an abstract base (`OnEnter()` / `Update()` / `SetParams()`);
each animation is its own class owning its own state — no file-scope
globals, no `switch` over bare functions. `LedController` owns the strip,
every pattern instance, and the currently-active one:

| Pattern | `LedPatternId` | Behavior |
|---|---|---|
| `OffPattern` | `kOff` (0) | All LEDs off |
| `FadeWaitPattern` | `kFadeWait` (1) | Fades to black, pauses, fades back in |
| `BreathePattern` | `kBreathe` (2) | Brightness bounces between low/high ("breathing") at the configured hue |
| `CometPattern` | `kComet` (3) | A lit pixel travels around the strip with a fading trail |
| `BlinkPattern` | `kBlink` (4) | Alternates the whole strip between a color and black |
| `EmergencyAlertPattern` | *(not selectable)* | Fast red strobe; only reachable via `EMERGENCY_COMMAND` or a physical e-stop round-trip through ROS, see below |

`SET_LED_PATTERN` payload fields (`hue`, `sat`, `val`, `speed_ms`) map onto
`LedPatternParams` and are reused by every pattern that needs them —
`speed_ms` means "tick/step interval," not a fixed magnitude, so the same
field tunes fade speed, breathing rate, comet speed, or blink rate depending
on which pattern is active.

**Hardcoded per-pattern defaults** (`DefaultParamsForPattern()` in
`lib/AutonavLed/include/autonav_led/led_pattern.h`) let a caller select a
pattern by id alone, without knowing FastLED hue values:

| `LedPatternId` | hue | sat | val | speed_ms | Meaning |
|---|---|---|---|---|---|
| `kFadeWait` (1) | 32 (amber) | 255 | 255 | 1200 | standby/waiting |
| `kBreathe` (2) | 128 (cyan) | 255 | 245 | 30 | idle/ready |
| `kComet` (3) | 160 (blue) | 255 | 255 | 30 | active/moving |
| `kBlink` (4) | 0 (red) | 255 | 255 | 200 | alert/attention |

These apply only when `SET_LED_PATTERN`'s payload is 1 byte (`pattern_id`
alone) — see the wire protocol table below for the 1-byte-vs-6-byte split.

`LedController::SetEmergency(true)` pushes `EmergencyAlertPattern` over
whatever was running and remembers it; `SetEmergency(false)` restores it.

## Wire protocol

Full spec lives in `lib/AutonavProtocol/include/autonav_protocol/protocol.h`
and must stay byte-for-byte identical to the ROS-side mirror at
`autonav_firmware/include/autonav_firmware/esp32_protocol.hpp`. Summary:

**Frame layout:**

| Offset | Field | Size | Value |
|---|---|---|---|
| 0 | SOF1 | 1 | `0xAA` |
| 1 | SOF2 | 1 | `0x55` |
| 2 | CMD | 1 | `CommandId` |
| 3 | LEN | 1 | payload length `N` (0–32) |
| 4 | PAYLOAD | `N` | command-specific |
| 4+N | CRC8 | 1 | `Crc8(CMD‖LEN‖PAYLOAD)` |

CRC-8/SMBUS (poly `0x07`, init `0x00`). On a CRC or length error the
decoder (`FrameDecoder`) drops the frame and resyncs from scratch rather
than discarding the whole input buffer, so it recovers from noise mid-stream
without needing a reconnect.

**Commands** (`autonav_protocol::CommandId`):

| Value | Name | Direction | Payload |
|---|---|---|---|
| `0x01` | `kHeartbeat` | ESP32→ROS, every 200 ms | `device_id[16]` + `estop_physical(1)` + `uptime_ms(4,LE)` |
| `0x02` | `kGetDeviceId` | ROS→ESP32 | — |
| `0x03` | `kDeviceIdResponse` | ESP32→ROS | `device_id[16]` |
| `0x04` | `kSetLedPattern` | ROS→ESP32 | `pattern_id(1)` **or** `pattern_id(1)+hue(1)+sat(1)+val(1)+speed_ms(2,LE)` |
| `0x05` | `kSetBrightness` | ROS→ESP32 | `brightness(1)` |
| `0x06` | `kEstopPhysicalPush` | ESP32→ROS, on edge | `state(1)` |
| `0x07` | `kEmergencyCommand` | ROS→ESP32 | `state(1)` |
| `0x08` | `kAck` | either | `orig_cmd(1)+status(1)` |

`device_id` is a fixed 16-byte, NUL-padded ASCII field (`DeviceIdentity`,
backed by NVS under the `"autonav"` namespace / `"device_id"` key — first
boot writes `board::DEVICE_ID_DEFAULT`).

`kSetLedPattern`'s payload length selects its meaning: exactly 1 byte
(`pattern_id` only) applies that pattern's hardcoded default params (table
above); exactly 6 bytes is a full `hue`/`sat`/`val`/`speed_ms` override.
Any other length gets `AckStatus::kErrorLength`.

Emergency feedback is intentionally round-tripped through ROS rather than
wired directly: a physical button press only ever sends
`kEstopPhysicalPush`; the ESP32's own LED strobe only ever comes from a
received `kEmergencyCommand`. The ROS-side `esp32_bridge_node` republishes
the physical push onto `/autonav/emergency_status` and is itself subscribed
to that same topic, so it forwards the emergency straight back down as
`kEmergencyCommand` — see that package's README for the full loop.

## Building and flashing

Requires [PlatformIO](https://platformio.org/) (CLI or the VS Code
extension).

```bash
cd autonav_firmware/firmware/Esp-32-Autonav-4.0

# Build only
pio run -e esp32dev

# Build + flash over USB
pio run -e esp32dev -t upload

# Serial monitor (matches board_config.h's UART_BAUD)
pio device monitor -b 115200
```

## Testing

The frame codec has a standalone Unity test suite (round-trip encode/decode
for every command, CRC rejection on a corrupted byte, and resync after
garbage containing a false start-of-frame byte):

```bash
pio test -e esp32dev
```

This flashes a small test binary to the board and runs it there — no
hardware peripherals (LEDs/e-stop) are required for it to pass, only a
connected ESP32.

## Recommended bring-up order

1. **Protocol only** — `pio test -e esp32dev`, then a UART loopback against
   the ROS-side `esp32_bridge_node` (confirm `GET_DEVICE_ID` →
   `DEVICE_ID_RESPONSE` round-trips via `pio device monitor`).
2. **Identity + heartbeat** — confirm `~/device_id` and `~/connected` on the
   ROS side track the board correctly, including on a cable unplug.
3. **LEDs** — verify each pattern visually via `ros2 service call` against
   `~/led/set_pattern` / `~/led/set_brightness` (`autonav_msgs/srv/SetLedPattern`
   / `SetBrightness` — see the ROS package's README for exact commands, and
   for the `/api/firmware/led/*` HTTP equivalents exposed by RobotServer, and
   that same README's "Frame-level debug logging" section for confirming
   frames/ACKs are actually round-tripping).

   If the strip is stuck on one color/brightness no matter what's sent —
   and the ROS-side debug log shows a clean `TX ...` / `RX ACK ... OK` for
   every call (i.e. the command genuinely reached the board and was
   applied without error) — the bug is downstream of the protocol, in
   `FastLED`'s config or the wiring, not in the command delivery. Narrow it
   down by testing the strip standalone first, with the UART link out of
   the picture entirely:
   ```cpp
   // Temporary, in setup() right after led.Begin() — confirms the strip
   // itself responds to FastLED before suspecting the protocol:
   led.SetPattern(LedPatternId::kBlink);
   ```
   If it still doesn't blink, check in this order: (a) **chipset mismatch**
   — `led_controller.cpp`'s `FastLED.addLeds<WS2812, ...>` assumes a
   3-channel WS2812/WS2812B strip; an RGBW strip (SK6812/NeoPixel RGBW)
   expects 32 bits/LED instead of 24 and will desync into
   stuck/garbage-looking colors under a `WS2812` driver; (b) **`LED_PIN`
   choice** — on ESP32 modules with PSRAM (WROVER), GPIO16/17 are wired
   internally to the PSRAM chip and are not safe general-purpose GPIOs;
   if your module has PSRAM, move `board::LED_PIN` off 16/17 (e.g. GPIO4);
   (c) common ground between the ESP32 and the strip's power supply.
4. **E-stop, both directions, last** — physical button press should flip
   `/autonav/emergency_status` and strobe the strip; a software-published
   `true` on that topic (button untouched) should strobe it too.
