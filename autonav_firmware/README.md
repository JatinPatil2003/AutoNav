# autonav_firmware

ROS2 (ament_cmake) package that owns every hardware-facing C++ component
that isn't the lidar or the IMU. It builds **two independent artifacts**
that don't know about each other:

1. **`AutonavInterface`** — a `ros2_control` `hardware_interface::SystemInterface`
   plugin that drives the two drive motors directly over RS485/Modbus.
2. **`esp32_bridge_node`** — a plain `rclcpp::Node` executable that owns the
   UART link to the `Esp-32-Autonav-4.0` peripheral board (device identity,
   LED patterns, physical e-stop feedback). It has nothing to do with motor
   control.

## Directory layout

```
autonav_firmware/
├── include/autonav_firmware/
│   ├── motor_controller.hpp         # RS485/Modbus motor driver (RMCS-3001)
│   ├── autonav_interface.hpp        # ros2_control SystemInterface plugin
│   ├── esp32_protocol.hpp           # UART frame protocol (mirrors the firmware's copy)
│   ├── esp32_frame_codec.hpp        # frame encoder/decoder
│   ├── esp32_serial_transport.hpp   # termios UART transport + reader thread
│   └── esp32_bridge_node.hpp        # the ESP32 bridge node
├── src/                             # matching .cpp files (+ main() in esp32_bridge_node.cpp)
├── launch/
│   └── esp32_bridge.launch.py
├── config/
│   └── esp32_bridge_params.yaml
├── firmware/                        # PlatformIO/Arduino ESP32 projects (flashed separately,
│   │                                 # not built by colcon) — see firmware/Esp-32-Autonav-4.0/README.md
│   ├── Esp-32-Autonav-4.0/          # current firmware (plain UART, no micro-ROS)
│   └── ...                          # older micro-ROS-based sketches, kept for reference only
├── docs/
│   └── RMCS-3001 V2.pdf             # motor driver datasheet (register map reference)
├── autonav_interface.xml            # pluginlib export for AutonavInterface
├── CMakeLists.txt
└── package.xml
```

Depends on the sibling `autonav_msgs` package (`../autonav_msgs`) for the
`SetLedPattern`/`SetBrightness` service definitions used by
`esp32_bridge_node` — see §2 below.

## 1. Motor control — `AutonavInterface` / `MotorController`

Motors are **not** driven through the ESP32 at all. `AutonavInterface`
(`include/autonav_firmware/autonav_interface.hpp`,
`src/autonav_interface.cpp`) is a `ros2_control` `SystemInterface` plugin
(exported via pluginlib as `autonav_firmware/AutonavInterface`, wired up in
`autonav_description/urdf/autonav_ros2_control.xacro`). It owns two
`MotorController` instances (`include/autonav_firmware/motor_controller.hpp`,
`src/motor_controller.cpp`), one per wheel, each talking Modbus RTU
(`libmodbus`) to an **RMCS-3001** drive (register map documented inline in
`motor_controller.hpp` and in `docs/RMCS-3001 V2.pdf`).

Hardware parameters (set in `autonav_ros2_control.xacro`):

| Param | Example | Meaning |
|---|---|---|
| `left_port` / `right_port` | `/dev/ttyUSB0` / `/dev/ttyACM0` | Serial device per wheel's Modbus adapter |
| `left_slave_id` / `right_slave_id` | `2` / `1` | Modbus slave address per wheel |

Each `MotorController` runs a dedicated worker thread
(`startWorker()`/`workerLoop()`) polling the drive every 5 ms and caching
velocity/position feedback in atomics; `AutonavInterface::read()/write()`
(called by the `controller_manager` update loop) only ever touch those
atomics, never block on Modbus I/O directly.

`AutonavInterface` also spins its own small internal `rclcpp::Node`
(`"autonav_emergency_listener"`) on a background thread, subscribed to
`/autonav/emergency_status` (`std_msgs/Bool`) — when `true`, it forces both
wheels into brake mode and refuses forwarded velocity commands until it
clears. That topic's actual publisher is the `esp32_bridge_node` below (via
the physical e-stop on the peripheral board), or anything else that
publishes to it.

A legacy, pre-Modbus variant (`include/autonav_firmware/autonav_interface_serial.hpp`,
`src/autonav_interface_serial.cpp`) that drove motors over plain serial
still exists on disk but is **not** referenced by `CMakeLists.txt` — dead
code kept for reference, not built.

## 2. ESP32 bridge — `esp32_bridge_node`

Owns the UART link to the `Esp-32-Autonav-4.0` peripheral board (see that
project's own README for the firmware side and the full protocol spec,
which must stay byte-identical to `esp32_protocol.hpp` here). Threading
mirrors `MotorController`: `Esp32SerialTransport` opens the port with raw
`termios` (8N1, blocking reads) and runs a dedicated reader thread that
feeds a byte-at-a-time `Esp32FrameDecoder` and invokes a callback — that
callback only ever touches mutex-guarded state/atomics, never publishes
directly. A 50 ms `rclcpp::TimerBase` on the node's own thread does the
actual publishing.

### Topics & services

| Name | Type | Direction | Purpose |
|---|---|---|---|
| `~/led/set_pattern` | `autonav_msgs/srv/SetLedPattern` | service | `pattern_id` (+ `use_custom_params`/`hue`/`sat`/`val`/`speed_ms`) → `SET_LED_PATTERN`; response is `{success, message}` from the ESP32's ACK |
| `~/led/set_brightness` | `autonav_msgs/srv/SetBrightness` | service | `brightness` (0–255) → `SET_BRIGHTNESS`; response is `{success, message}` from the ESP32's ACK |
| `/autonav/emergency_status` | `std_msgs/Bool` | pub **and** sub | Published from the physical e-stop; subscribed so any software-triggered emergency (including its own publish) is forwarded back down to the ESP32 as `EMERGENCY_COMMAND` |
| `~/connected` | `std_msgs/Bool` | pub | True iff the serial port is open **and** a heartbeat arrived within `heartbeat_timeout_sec` **and** (if `expected_device_id` is set) it matches |
| `~/device_id` | `std_msgs/String` | pub | Latest device ID confirmed via heartbeat/`GET_DEVICE_ID` |
| `~/get_device_id` | `std_srvs/Trigger` | service | Returns the cached ID immediately if known, otherwise queries the board and waits up to 500 ms |

`SetLedPattern`/`SetBrightness` are services rather than topics specifically
so a caller gets a definite success/failure result instead of firing a
command into the void — the node sends the frame and blocks (up to 500 ms,
`sendAndWaitForAck()`) for the ESP32's `ACK` before replying. Both are
defined in the small `autonav_msgs` interfaces package (sibling to this
one, `srv/SetLedPattern.srv` / `srv/SetBrightness.srv`) rather than reusing
`std_msgs`, since a service needs a named response type to carry
`success`/`message` back — `std_srvs/Trigger` doesn't have fields for the
request side. `/autonav/emergency_status` stays a plain `std_msgs/Bool`
topic since nothing there needs a round-trip reply.

`SetLedPattern.request.use_custom_params` decides what goes out over the
wire: `false` sends just `pattern_id` (1-byte payload), and the ESP32
applies that pattern's hardcoded default color/speed
(`DefaultParamsForPattern()` — see the firmware's README for the table);
`true` sends the full 6-byte payload with the request's `hue`/`sat`/`val`/
`speed_ms`. There's no partial-override mode — the wire protocol only has
these two payload lengths, so a caller either picks a pattern by id alone
or supplies all four params together.

### Parameters (`config/esp32_bridge_params.yaml`)

| Param | Default | Meaning |
|---|---|---|
| `serial_port` | `/dev/esp` | UART device for the ESP32 |
| `baud_rate` | `115200` | Must match `board::UART_BAUD` in the firmware |
| `expected_device_id` | `AUTONAV-001` | Must match `board::DEVICE_ID_DEFAULT` in the firmware (or whatever was later persisted to NVS); empty string accepts any ID |
| `heartbeat_timeout_sec` | `1.0` | How long without a heartbeat before `~/connected` goes false |

### Running it

```bash
ros2 launch autonav_firmware esp32_bridge.launch.py
```

Already included from `autonav_bringup/launch/autonav_bringup.launch.py`
alongside the IMU and lidar includes — no extra step needed for a full
bringup.

### Frame-level debug logging

`Esp32SerialTransport` (the class that owns the raw UART fd and reader
thread) logs every frame it sends and receives, keyed on the logger name
`esp32_serial_transport`: `RCLCPP_DEBUG` for a normal TX/RX (command name,
payload length, hex payload), `RCLCPP_WARN` for a dropped CRC- or
length-corrupted frame. Debug is off by default (would spam at every 200 ms
heartbeat); turn it on for a single run to confirm whether a command is
actually reaching the ESP32 and whether its ACK is coming back:

```bash
# Add -p serial_port:=/dev/ttyUSBx if your board isn't on the default /dev/esp
ros2 run autonav_firmware esp32_bridge_node --ros-args --log-level esp32_serial_transport:=debug
```

If `TX SET_LED_PATTERN ...` and a matching `RX ACK ...` both appear but the
strip doesn't change, the frame link is fine and the problem is downstream
of it (FastLED config, wiring, or the LED hardware itself) — see the
firmware README's bring-up notes.

### Manual verification & LED service calls

#### Link health & device identity

```bash
ros2 topic hz /esp32_bridge_node/device_id
ros2 topic echo /esp32_bridge_node/device_id
ros2 service call /esp32_bridge_node/get_device_id std_srvs/srv/Trigger
```

#### LED Pattern Reference (`/esp32_bridge_node/led/set_pattern`)

Service type: `autonav_msgs/srv/SetLedPattern`

| `pattern_id` | Name | Default Appearance | Robot Meaning |
|---|---|---|---|
| `0` | `kOff` | Strip is dark | Powered off / LEDs disabled |
| `1` | `kFadeWait` | Amber breathing fade, long pause | Standby / waiting |
| `2` | `kBreathe` | Slow cyan breathing | Idle / ready |
| `3` | `kComet` | Blue chasing comet | Active / moving |
| `4` | `kBlink` | Fast red blink | Alert / attention / warning |

##### Using pattern defaults (`use_custom_params: false`)
When `use_custom_params` is `false`, the ESP32 automatically applies its built-in default color and speed for the pattern:

```bash
# Set pattern to Breathe (Cyan, Ready/Idle)
ros2 service call /esp32_bridge_node/led/set_pattern autonav_msgs/srv/SetLedPattern "{pattern_id: 2, use_custom_params: false}"

# Set pattern to Comet (Blue chasing, Moving/Active)
ros2 service call /esp32_bridge_node/led/set_pattern autonav_msgs/srv/SetLedPattern "{pattern_id: 3, use_custom_params: false}"

# Set pattern to FadeWait (Amber standby)
ros2 service call /esp32_bridge_node/led/set_pattern autonav_msgs/srv/SetLedPattern "{pattern_id: 1, use_custom_params: false}"

# Set pattern to Blink (Fast red alert)
ros2 service call /esp32_bridge_node/led/set_pattern autonav_msgs/srv/SetLedPattern "{pattern_id: 4, use_custom_params: false}"

# Turn LEDs off
ros2 service call /esp32_bridge_node/led/set_pattern autonav_msgs/srv/SetLedPattern "{pattern_id: 0, use_custom_params: false}"
```

##### Using custom HSV & speed (`use_custom_params: true`)
You can supply custom FastLED HSV values (`hue`: 0–255, `sat`: 0–255, `val`: 0–255) and animation speed (`speed_ms` in milliseconds):

```bash
# Green breathing pattern (hue 96 = green, speed 30 ms)
ros2 service call /esp32_bridge_node/led/set_pattern autonav_msgs/srv/SetLedPattern \
  "{pattern_id: 2, use_custom_params: true, hue: 96, sat: 255, val: 255, speed_ms: 30}"

# Purple blinking pattern (hue 192 = purple, speed 500 ms)
ros2 service call /esp32_bridge_node/led/set_pattern autonav_msgs/srv/SetLedPattern \
  "{pattern_id: 4, use_custom_params: true, hue: 192, sat: 255, val: 255, speed_ms: 500}"
```

#### LED Brightness Reference (`/esp32_bridge_node/led/set_brightness`)

Service type: `autonav_msgs/srv/SetBrightness` (`brightness`: 0–255)

```bash
# Set brightness to ~50% (128)
ros2 service call /esp32_bridge_node/led/set_brightness autonav_msgs/srv/SetBrightness "{brightness: 128}"

# Set brightness to maximum (255)
ros2 service call /esp32_bridge_node/led/set_brightness autonav_msgs/srv/SetBrightness "{brightness: 255}"
```

#### Emergency Status

```bash
# Software-triggered emergency (no physical button needed)
ros2 topic pub /autonav/emergency_status std_msgs/msg/Bool "{data: true}" --once
ros2 topic echo /autonav/emergency_status
```

Both service calls print the ESP32's actual `{success, message}` response,
so a CRC/length-rejected command or a disconnected board shows up directly
in the call's output instead of silently doing nothing.

### Testing from RobotServer (HTTP API)

`RobotServer` (the Node/rclnodejs HTTP bridge) exposes these same two
services as REST endpoints — see `RobotServer/src/modules/firmware/` and
`RobotServer/src/ros/rosServiceGateway.ts` (the generic service-call
gateway `FirmwareService` calls through):

```bash
curl -X POST http://<robot-host>:<port>/api/firmware/led/pattern \
  -H "Content-Type: application/json" \
  -d '{"patternId": 2, "hue": 86, "sat": 255, "val": 255, "speedMs": 50}'

curl -X POST http://<robot-host>:<port>/api/firmware/led/brightness \
  -H "Content-Type: application/json" \
  -d '{"brightness": 180}'
```

Both return the same `{success, message}` shape as the ROS service, as
JSON, with HTTP 502 if the ESP32 didn't ACK. `autonav_msgs` must be built
and sourced wherever `esp32_bridge_node` and `RobotServer`'s ROS node run —
see that package's note in `RobotServer/README.md`'s "Known gaps".

## Building

Standard colcon package; depends on `rclcpp`, `std_msgs`, `std_srvs`,
`autonav_msgs`, `hardware_interface`, `pluginlib`, `rclcpp_lifecycle`, and
`libmodbus-dev` (system package, found via `pkg-config`).

```bash
colcon build --packages-select autonav_msgs autonav_firmware
```

This builds both artifacts: the `autonav_firmware` shared library (the
`ros2_control` plugin) and the `esp32_bridge_node` executable, independently
of each other — neither depends on the other being present or running.
