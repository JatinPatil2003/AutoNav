import { config } from "../../config/env";
import { logger } from "../../common/utils/logger.util";
import { IRobotRepository } from "../robots/robot.repository.interface";
import { IRobotNotifier } from "./robotNotifier.interface";

/**
 * PlatformServer calling a robot's own RobotServer directly over the VPN
 * by vpnHostname, to push a best-effort roster-invalidate signal after a
 * grant/revoke. No auth of its own — the VPN is the trust boundary (see
 * root README) — and failures are the caller's concern to swallow;
 * RobotServer self-corrects via its own periodic resync regardless.
 */
export class HttpRobotNotifier implements IRobotNotifier {
  constructor(private readonly robotRepository: IRobotRepository) {}

  async notifyRosterInvalidate(robotId: string): Promise<void> {
    const robot = await this.robotRepository.findByRobotId(robotId);
    if (!robot) {
      logger.warn({ robotId }, "HttpRobotNotifier: robot not found, skipping push");
      return;
    }

    const url = `${config.robotCall.scheme}://${robot.vpnHostname}/api/platform-auth/roster/invalidate`;

    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), config.robotCall.timeoutMs);

    try {
      const response = await fetch(url, {
        method: "POST",
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
