import { ServiceGateway } from "./rosServiceGateway";
import { config } from "../config/env";
import { logger } from "../common/utils/logger.util";

const GET_DEVICE_ID_SERVICE = "/esp32_bridge_node/get_device_id";
const SERVICE_CALL_TIMEOUT_MS = 10_000;

interface TriggerResponse {
  success: boolean;
  message: string;
}

/**
 * Resolves this robot's own identity from the ESP32 bridge's cached device
 * id (the same value it publishes on ~/device_id, via its ~/get_device_id
 * Trigger service) — never configured statically, so the only way to act
 * as a given robot is to physically be the ESP32 it's wired to. Replaces
 * the old static PLATFORM_ROBOT_SECRET: PlatformClient and the local-user
 * cache both key off whatever this resolves to.
 */
export class DeviceIdentityService {
  private robotId: string | null = null;

  constructor(private readonly serviceGateway: ServiceGateway) {}

  getRobotId(): string | null {
    return this.robotId;
  }

  /** Idempotent — returns the cached id immediately once resolved. */
  async resolve(): Promise<string | null> {
    if (this.robotId) return this.robotId;

    try {
      const response = await this.serviceGateway.call<Record<string, never>, TriggerResponse>(
        "std_srvs/srv/Trigger",
        GET_DEVICE_ID_SERVICE,
        {},
        SERVICE_CALL_TIMEOUT_MS,
      );
      if (response.success && response.message) {
        this.robotId = response.message;
        logger.info({ robotId: this.robotId }, "deviceIdentity: resolved from ESP32 bridge");
        return this.robotId;
      }
      logger.warn({ response }, "deviceIdentity: get_device_id returned no device id yet");
    } catch (err) {
      logger.warn({ err }, "deviceIdentity: get_device_id service call failed");
    }

    if (!config.isProduction && config.devDeviceIdOverride) {
      this.robotId = config.devDeviceIdOverride;
      logger.warn(
        { robotId: this.robotId },
        "deviceIdentity: using DEV_DEVICE_ID_OVERRIDE (non-production fallback, no ESP32 attached)",
      );
      return this.robotId;
    }

    return null;
  }

  /** Blocking resolution with a few retries, for use at boot before the server starts accepting traffic. */
  async resolveWithRetries(attempts: number): Promise<string | null> {
    for (let attempt = 1; attempt <= attempts; attempt += 1) {
      const robotId = await this.resolve();
      if (robotId) return robotId;
      if (attempt < attempts) {
        logger.warn({ attempt, attempts }, "deviceIdentity: retrying device id resolution");
      }
    }
    return null;
  }
}
