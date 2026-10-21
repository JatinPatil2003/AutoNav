# RobotServer

The robot-side backend: the only process that bridges the ROS 2 graph to HTTP/WebSocket/WebRTC for the operator apps (PlatformApp's legacy robot UI, AndroidApp, LinuxApp) and, as of this change, the local RobotApp HMI PWA it now serves and authenticates directly. Replaces the old `WebServer/` (FastAPI + rclpy).

## Stack

Node.js + TypeScript + Express, deliberately matching `PlatformServer`'s architecture (layered `routes → controller → service → repository interface → repository → model`, composition root in `src/app.ts`, zod validation, Mongoose, pino logging, swagger-jsdoc docs, jest + fakes for unit tests).

ROS 2 access is via [`rclnodejs`](https://github.com/RobotWebTools/rclnodejs) rather than Python's `rclpy`. That's a real trade-off, made deliberately for stack consistency with the rest of the repo — see the root project history for the alternatives considered. `rclnodejs`'s 2.x line supports ROS 2 Lyrical (as well as Kilted/Jazzy/Humble/Rolling) — this repo targets Lyrical, matching `docker/base.Dockerfile` and `RobotServer/Dockerfile`.

## Architecture

```
src/
├── index.ts              # entrypoint: connects Mongo, starts the ROS node, starts the HTTP+WS server
├── app.ts                # composition root — every gateway/repository/service/controller wired here
├── config/env.ts         # zod-validated env config
├── core/
│   ├── eventBus.ts        # decouples the ROS layer from anything reacting to ROS data (today: telemetry WS)
│   └── wsRouter.ts         # routes WebSocket upgrades on one HTTP server by pathname
├── ros/                   # the ROS integration layer — nothing outside this folder imports rclnodejs directly
│   ├── rosNode.ts          # owns the rclnodejs Node lifecycle
│   ├── rosTopicGateway.ts  # generic pub/sub for ANY topic, keyed by topic name
│   ├── rosServiceGateway.ts # generic service calls for ANY service, keyed by service name
│   ├── rosActionGateway.ts  # generic action calls for ANY action, keyed by action name
│   ├── quaternion.ts       # pure yaw<->quaternion helpers
│   ├── launchManager.ts    # single source of truth for `ros2 launch` subprocesses, mutually exclusive
│   └── runOnce.ts          # short-lived commands (map_saver_cli), distinct from launchManager
├── db/connection.ts
└── modules/
    ├── teleop/             # manual drive, emergency stop — REST + /ws/teleop
    ├── navigation/          # Nav2 goals, saved poses, localize-at-point — REST
    ├── mapping/             # SLAM session lifecycle, waypoint capture, save-map workflow — REST
    ├── telemetry/           # /ws/telemetry broadcast (map/location/nav-status); mappers.ts + broadcaster.ts
    ├── video/               # /ws/webrtc camera signaling
    ├── firmware/             # ESP32 peripheral board (LED pattern/brightness) — REST, via autonav_firmware's esp32_bridge_node services
    ├── goals/, maps/         # shared persistence (Mongoose + GridFS) used by navigation & mapping
    └── system/               # network/diagnostics/maintenance — see "Known gaps" below
```

ROS access is organized by **communication primitive**, not by feature:
`RosTopicGateway`/`RosServiceGateway`/`RosActionGateway` (`src/ros/`) are the
only three classes that ever call `rclnodejs`'s `createPublisher`/
`createSubscription`/`createClient`/`ActionClient` — every module's service
(`TeleopService`, `NavigationService`, `MappingService`, `FirmwareService`)
depends on whichever of the three it needs directly, by name
(e.g. `topicGateway.publish("geometry_msgs/msg/Twist", "cmd_vel", ...)`),
instead of each feature owning its own gateway class and interface. Tests
substitute `tests/unit/fakes/FakeRosTopicGateway`/`FakeRosServiceGateway`/
`FakeRosActionGateway` for the same 3 classes — no ROS node needed for unit
tests, same as before.

## What changed vs. the old WebServer (and why)

- **Mongo URI from env**, validated at boot — the old `WebServer/mongodb/db.py` had a hardcoded Atlas connection string with plaintext credentials committed to the repo.
- **One `LaunchManager`** instead of three independent `Popen`/`killpg` globals (bringup/nav/slam) with no mutual exclusion — nothing used to stop you from launching navigation and SLAM at the same time.
- **ROS → WebSocket via `EventBus`**, not `asyncio.run()` called on every single ROS callback tick (the old telemetry timer fired at 20Hz, spinning up and tearing down a fresh event loop every time).
- **Routes consolidated.** The old API had parallel near-duplicates (`/navigation/list/maps` vs `/navigation/list/maps_linux`, `use_map` vs `use_map_linux`, three different `save_map_data` handlers shadowing each other in the same file). This rewrite has one `/api/navigation/*` contract — see "Migrating clients" below.
- **System/admin endpoints (WiFi, VPN, reboot, diagnostics, firmware version/update, credentials, DB reset) are stubbed (501), not ported.** These shelled out to `nmcli`/`systemctl`/`docker` on the old WebServer and are being redesigned, not carried over as-is. The route shape is fixed so clients have a stable contract to build against; `modules/system/*` is where the real implementation lands next.
- **No auth was added.** The robot has no auth of its own either way — the OpenVPN tunnel is the trust boundary, same as before. CORS is now an explicit allow-list from `CORS_ORIGINS` instead of `*` + credentials (an invalid/unsafe combination the old server shipped with).

## Known gaps — verify before relying on these

1. **`rclnodejs` call shapes** in `ros/rosTopicGateway.ts`, `ros/rosServiceGateway.ts`, `ros/rosActionGateway.ts` (`createPublisher`/`createSubscription`/`createTimer`/`createClient`/`ActionClient.sendGoal`) match its long-stable documented API, but still need a pass against a live ROS graph on real hardware. `scripts/ros-smoke-test.ts` (`npm run smoke:ros`) exists for exactly this — it exercises `get_device_id`, `SetLedPattern`/`SetBrightness` (field names already hand-verified against `autonav_msgs/srv/*.srv`), `SetInitialPose`, `NavigateToPose` server reachability, and `/map`/`/amcl_pose` subscription QoS, and reports pass/fail per shape. Run it against the real robot and update this note once confirmed — it's a manual script, deliberately never run in CI.
2. **TF2 lookup during SLAM — resolved.** `rclnodejs` has no TF2 binding, so AMCL-only pose during SLAM (when AMCL isn't running) used to leave `MappingService.getCurrentLocation()`/`captureWaypoint()` with nothing to read. `autonav_utils`'s `tf_pose_bridge` node now republishes the `map`→`base_footprint` TF as a plain `/pose_bridge/map_pose` topic during both SLAM backends (`slam.launch.py` and `cartographer.launch.py`), which `MappingService`/`telemetry.broadcaster.ts` prefer over AMCL whenever a mapping launch is active.
3. **`modules/video/ffmpegVideoSource.ts` is not implemented.** The old WebServer used aiortc + OpenCV (`cv2.VideoCapture` → raw frames → `av.VideoFrame`), which has no direct equivalent confirmed on `werift` (the Node WebRTC library picked for this rewrite). The WebRTC *signaling* (`video.wsServer.ts`) is implemented and mirrors the old offer/answer/ICE flow; only the camera→track pipeline is a stub.

## Running locally

```bash
cp .env.example .env
npm install   # needs a sourced ROS 2 environment — rclnodejs builds native bindings against it
npm run dev    # nodemon + ts-node, http://localhost:8000
npm run build  # tsc -> dist/
npm test        # jest, unit tests only (fakes — no ROS/Mongo needed)
npm run lint
```

Interactive API docs at `GET /api/docs`.
