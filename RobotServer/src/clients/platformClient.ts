import { config } from "../config/env";
import { DeviceIdentityService } from "../ros/deviceIdentity";
import { IPlatformClient, RosterEntryDto } from "./platformClient.interface";

export { RosterEntryDto } from "./platformClient.interface";

const REQUEST_TIMEOUT_MS = 10_000;

/**
 * This robot's calls into PlatformServer's /api/robot-auth/* API, now
 * reachable from the public internet rather than only over the VPN. Each
 * call identifies itself with its own device id (resolved from the ESP32
 * bridge, see ros/deviceIdentity.ts) and authenticates with the same
 * ROBOT_LINK_SECRET used for /ws/robot-link, as `Authorization: Robot <secret>`.
 */
export class PlatformClient implements IPlatformClient {
  constructor(private readonly deviceIdentity: DeviceIdentityService) {}

  async checkUserExists(email: string): Promise<boolean> {
    const body = await this.requestJson<{ exists: boolean }>("POST", "/api/robot-auth/check-user", {
      email,
    });
    return body.exists;
  }

  async fetchRoster(): Promise<RosterEntryDto[]> {
    const body = await this.requestJson<{ users: RosterEntryDto[] }>("GET", "/api/robot-auth/roster");
    return body.users;
  }

  async requestPinSetupOtp(email: string): Promise<void> {
    await this.requestJson("POST", "/api/robot-auth/pin-setup/request-otp", { email });
  }

  async verifyPinSetupOtp(email: string, otp: string): Promise<{ pinSetupToken: string }> {
    return this.requestJson("POST", "/api/robot-auth/pin-setup/verify-otp", { email, otp });
  }

  async confirmPinSetup(pinSetupToken: string, pin: string): Promise<RosterEntryDto> {
    const body = await this.requestJson<{ entry: RosterEntryDto }>(
      "POST",
      "/api/robot-auth/pin-setup/confirm",
      { pinSetupToken, pin },
    );
    return body.entry;
  }

  private async requestJson<T>(method: string, path: string, body?: unknown): Promise<T> {
    const robotId = this.deviceIdentity.getRobotId();
    if (!robotId) {
      throw new Error("platformClient: device id not resolved yet, cannot call PlatformServer");
    }

    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), REQUEST_TIMEOUT_MS);

    let response: Response;
    try {
      response = await fetch(`${config.platform.baseUrl}${path}`, {
        method,
        headers: {
          "Content-Type": "application/json",
          "x-robot-id": robotId,
          authorization: `Robot ${config.robotLinkSecret}`,
        },
        body: body ? JSON.stringify(body) : undefined,
        signal: controller.signal,
      });
    } finally {
      clearTimeout(timeout);
    }

    if (!response.ok) {
      const text = await response.text().catch(() => "");
      throw new Error(`platformClient: ${method} ${path} failed with ${response.status}: ${text}`);
    }

    return response.json() as Promise<T>;
  }
}
