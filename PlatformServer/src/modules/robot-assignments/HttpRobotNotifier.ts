import { config } from "../../config/env";
import { logger } from "../../common/utils/logger.util";
import { decryptSecret } from "../../common/utils/secretCrypto.util";
import { signPlatformToRobotToken } from "../../common/utils/jwt.util";
import { IRobotRepository } from "../robots/robot.repository.interface";
import { IRobotNotifier } from "./robotNotifier.interface";

/**
 * Direction 2 of the service-to-service JWT design: PlatformServer signs a
 * fresh, short-lived token using *that specific robot's* own plaintext
 * secret as the HMAC key, then calls its RobotServer directly over the VPN
 * by vpnHostname. No session/refresh machinery — each call mints its own
 * token. Failures are the caller's concern to swallow (this is always a
 * best-effort push; RobotServer self-corrects via its own periodic resync).
 */
export class HttpRobotNotifier implements IRobotNotifier {
  constructor(private readonly robotRepository: IRobotRepository) {}

  async notifyRosterInvalidate(robotId: string): Promise<void> {
    const robot = await this.robotRepository.findByRobotId(robotId);
    if (!robot) {
      logger.warn({ robotId }, "HttpRobotNotifier: robot not found, skipping push");
      return;
    }

    const secret = decryptSecret(robot.robotSecretEncrypted);
    const token = signPlatformToRobotToken(secret, robot.robotId);
    const url = `${config.robotCall.scheme}://${robot.vpnHostname}/api/platform-auth/roster/invalidate`;

    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), config.robotCall.timeoutMs);

    try {
      const response = await fetch(url, {
        method: "POST",
        headers: { Authorization: `Bearer ${token}` },
        signal: controller.signal,
      });
      if (!response.ok) {
        logger.warn(
          { robotId, status: response.status },
          "HttpRobotNotifier: robot rejected roster-invalidate push",
        );
      }
    } finally {
      clearTimeout(timeout);
    }
  }
}
