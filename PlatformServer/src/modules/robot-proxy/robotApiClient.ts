import { RobotUnreachableError } from "../../common/errors";
import { config } from "../../config/env";
import { logger } from "../../common/utils/logger.util";
import { IRobotDocument } from "../robots/robot.model";
import { IRobotApiClient } from "./robotApiClient.interface";

/**
 * Same call pattern HttpRobotNotifier already uses (vpnHostname, no auth of
 * its own — the VPN is the trust boundary, exactly as that one-off push
 * already relies on) generalized into a reusable client, since the Phase 3
 * decision was to proxy request/response navigation+mapping commands as
 * plain REST over the VPN rather than relay them over the robot-link
 * WebSocket (that socket is reserved for streaming — pose/map/nav-status).
 */
export class HttpRobotApiClient implements IRobotApiClient {
  async request<TResponse>(
    robot: IRobotDocument,
    method: "GET" | "POST",
    path: string,
    body?: unknown,
  ): Promise<TResponse> {
    const url = `${config.robotCall.scheme}://${robot.vpnHostname}${path}`;

    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), config.robotCall.timeoutMs);

    try {
      const response = await fetch(url, {
        method,
        headers: body !== undefined ? { "Content-Type": "application/json" } : undefined,
        body: body !== undefined ? JSON.stringify(body) : undefined,
        signal: controller.signal,
      });

      if (!response.ok) {
        logger.warn({ robotId: robot.robotId, path, status: response.status }, "HttpRobotApiClient: robot rejected request");
        throw new RobotUnreachableError(`Robot responded with ${response.status}`);
      }

      return (await response.json()) as TResponse;
    } catch (err) {
      if (err instanceof RobotUnreachableError) throw err;
      logger.warn({ err, robotId: robot.robotId, path }, "HttpRobotApiClient: request failed");
      throw new RobotUnreachableError("Robot did not respond");
    } finally {
      clearTimeout(timeout);
    }
  }
}
