import { config } from "../config/env";
import { linkSecretStore } from "../config/linkSecretStore";
import { AppError } from "../common/errors";
import { DeviceIdentityService } from "../ros/deviceIdentity";
import { IPlatformClient, RosterEntryDto } from "./platformClient.interface";

export { RosterEntryDto } from "./platformClient.interface";

// 10s was tight even for localhost; now that these calls cross the public
// internet (Cloudflare + nginx, see PLATFORM_BASE_URL), give them real margin.
const REQUEST_TIMEOUT_MS = 25_000;

// A brief drop on the robot's own internet uplink (observed as undici's
// connect-timeout failing on every Cloudflare address, v4 and v6 alike) is
// common enough on-site that failing an entire PIN-setup/login attempt over
// one blip is bad UX. One retry after a short pause covers that without
// masking a real, sustained outage.
const NETWORK_FAILURE_RETRY_DELAY_MS = 1_500;

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

  async rotateLinkSecret(): Promise<{ linkSecret: string }> {
    return this.requestJson("POST", "/api/robot-auth/rotate-link-secret");
  }

  private async requestJson<T>(method: string, path: string, body?: unknown, isRetry = false): Promise<T> {
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
          authorization: `Robot ${linkSecretStore.getSecret()}`,
        },
        body: body ? JSON.stringify(body) : undefined,
        signal: controller.signal,
      });
    } catch (err) {
      // TypeError here means fetch never got a response at all (DNS/connect
      // failure, including undici's own connect-timeout) — a network-level
      // problem, not an application error. A manual abort (our own 25s
      // ceiling) surfaces as a DOMException, not a TypeError, and is never
      // retried since that long with no response means something is
      // actually stuck, not a passing blip.
      if (!isRetry && err instanceof TypeError) {
        await new Promise((resolve) => setTimeout(resolve, NETWORK_FAILURE_RETRY_DELAY_MS));
        return this.requestJson<T>(method, path, body, true);
      }
      throw err;
    } finally {
      clearTimeout(timeout);
    }

    if (!response.ok) {
      const text = await response.text().catch(() => "");
      const parsed = (() => {
        try {
          return JSON.parse(text) as { message?: string };
        } catch {
          return undefined;
        }
      })();
      // 4xx from PlatformServer (bad/expired OTP, unknown user, etc.) is a
      // client-facing error this robot should pass straight through to the
      // HMI with its real status and message — not swallow into an opaque
      // 500. 5xx/network issues stay a plain Error (caller logs + 500s).
      if (response.status >= 400 && response.status < 500) {
        throw new AppError(
          parsed?.message ?? `platformClient: ${method} ${path} failed with ${response.status}`,
          response.status,
        );
      }
      throw new Error(`platformClient: ${method} ${path} failed with ${response.status}: ${text}`);
    }

    return response.json() as Promise<T>;
  }
}
