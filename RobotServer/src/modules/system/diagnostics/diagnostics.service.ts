import fs from "node:fs";
import path from "node:path";
import { NotFoundError } from "../../../common/errors";
import { DeviceIdentityService } from "../../../ros/deviceIdentity";
import { LaunchManager } from "../../../ros/launchManager";
import { TopicGateway } from "../../../ros/rosTopicGateway";
import { TopicFrequencyMonitor } from "../../../ros/topicFrequencyMonitor";
import { BatteryStatusResult, FirmwareService } from "../../firmware/firmware.service";
import { MappingService } from "../../mapping/mapping.service";

const ESP32_CONNECTED_TOPIC = "/esp32_bridge_node/connected";

/** Real topic names, confirmed from each driver's own source rather than guessed — see autonav_lidar/rplidar_node.cpp, autonav_imu's SensorService.py (prefix param "autonav_imu/"), and autonav_localization's EKF/cartographer.launch.py's /odom->/odometry/filtered remap. */
const SENSOR_TOPICS: Record<string, { type: string; topicName: string }> = {
  lidar: { type: "sensor_msgs/msg/LaserScan", topicName: "/scan" },
  imu: { type: "sensor_msgs/msg/Imu", topicName: "autonav_imu/imu" },
  odom: { type: "nav_msgs/msg/Odometry", topicName: "/odometry/filtered" },
};

export interface SensorHealth {
  sensorName: string;
  hz: number | null;
  lastMessageAgeMs: number | null;
}

export interface SystemStatus {
  robotId: string | null;
  rclOk: boolean;
  esp32Connected: boolean;
  activeLaunch: string | null;
  activeSlamBackend: string | null;
}

export interface VersionInfo {
  robotServerVersion: string;
  esp32FirmwareVersion: null;
  esp32FirmwareVersionNote: string;
}

interface Esp32ConnectedMsg {
  data: boolean;
}

/**
 * Backs /api/system/diagnostics. Reports what's actually knowable today —
 * see getVersion()'s doc comment for why ESP32 firmware version is null,
 * and FirmwareService.getBatteryStatus()/isDefault for why battery is a
 * placeholder until the board has a real voltage-sense ADC.
 */
export class DiagnosticsService {
  constructor(
    private readonly topicGateway: TopicGateway,
    private readonly frequencyMonitor: TopicFrequencyMonitor,
    private readonly deviceIdentity: DeviceIdentityService,
    private readonly launchManager: LaunchManager,
    private readonly mappingService: MappingService,
    private readonly firmwareService: FirmwareService,
  ) {
    for (const { type, topicName } of Object.values(SENSOR_TOPICS)) {
      this.frequencyMonitor.track(type, topicName);
    }
    this.topicGateway.subscribe<Esp32ConnectedMsg>("std_msgs/msg/Bool", ESP32_CONNECTED_TOPIC);
  }

  getSensorHealth(sensorName: string): SensorHealth {
    const sensor = SENSOR_TOPICS[sensorName];
    if (!sensor) {
      throw new NotFoundError(`Unknown sensor "${sensorName}" — known sensors: ${Object.keys(SENSOR_TOPICS).join(", ")}`);
    }
    return {
      sensorName,
      hz: this.frequencyMonitor.getHz(sensor.topicName),
      lastMessageAgeMs: this.frequencyMonitor.getLastMessageAgeMs(sensor.topicName),
    };
  }

  getStatus(): SystemStatus {
    const esp32Connected = this.topicGateway.getLatest<Esp32ConnectedMsg>(ESP32_CONNECTED_TOPIC)?.data ?? false;
    return {
      robotId: this.deviceIdentity.getRobotId(),
      rclOk: this.topicGateway.isNodeAlive(),
      esp32Connected,
      activeLaunch: this.launchManager.status().name ?? null,
      activeSlamBackend: this.mappingService.getActiveBackend(),
    };
  }

  /**
   * The ESP32 wire protocol has no GET_FIRMWARE_VERSION command (confirmed
   * absent in autonav_firmware/esp32_protocol.hpp) — there is no ESP32
   * firmware version to report. What IS reported is RobotServer's own
   * package version, read directly off disk rather than via a static
   * `import ... from "../../../../package.json"`, since that import
   * crosses outside tsconfig's rootDir ("src") and tsc rejects it.
   */
  getVersion(): VersionInfo {
    return {
      robotServerVersion: readRobotServerVersion(),
      esp32FirmwareVersion: null,
      esp32FirmwareVersionNote:
        "The ESP32 firmware's wire protocol has no GET_FIRMWARE_VERSION command yet — this is RobotServer's own software version, not the ESP32's.",
    };
  }

  getBatteryStatus(): Promise<BatteryStatusResult> {
    return this.firmwareService.getBatteryStatus();
  }
}

let cachedVersion: string | undefined;

function readRobotServerVersion(): string {
  if (cachedVersion) return cachedVersion;
  try {
    const packageJsonPath = path.join(__dirname, "..", "..", "..", "..", "package.json");
    const raw = fs.readFileSync(packageJsonPath, "utf-8");
    cachedVersion = (JSON.parse(raw) as { version: string }).version;
  } catch {
    cachedVersion = "unknown";
  }
  return cachedVersion;
}
