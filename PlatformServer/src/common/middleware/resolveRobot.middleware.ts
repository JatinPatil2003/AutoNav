import crypto from "node:crypto";
import { NextFunction, Request, Response } from "express";
import { NotFoundError, UnauthorizedError } from "../errors";
import { IRobotRepository } from "../../modules/robots/robot.repository.interface";
import { asyncHandler } from "../utils/asyncHandler.util";

/**
 * Identifies and authenticates the calling robot for /api/robot-auth/*,
 * now reachable from the public internet (see AWS/nginx/platform-server.autonav.in.conf)
 * rather than only over the VPN. The robot sends its own device id plus the
 * same ROBOT_LINK_SECRET it uses for /ws/robot-link, as `Authorization: Robot <secret>`;
 * compared with crypto.timingSafeEqual against the stored sha256 hash so a
 * timing side-channel can't leak the secret byte-by-byte (same pattern as
 * robotLink.gateway.ts's authenticate()).
 */
export function createResolveRobot(robotRepository: IRobotRepository) {
  return asyncHandler(async (req: Request, _res: Response, next: NextFunction): Promise<void> => {
    const robotId = req.header("x-robot-id");
    if (!robotId) {
      throw new UnauthorizedError("x-robot-id header is required");
    }

    const authHeader = req.header("authorization");
    const match = authHeader?.match(/^Robot (.+)$/);
    if (!match) {
      throw new UnauthorizedError("Authorization: Robot <secret> header is required");
    }
    const presentedSecret = match[1];

    const robot = await robotRepository.findByRobotIdWithLinkSecret(robotId);
    if (!robot) {
      throw new NotFoundError("Unknown robot");
    }
    if (!robot.linkSecretHash) {
      throw new UnauthorizedError("Robot has no link secret provisioned");
    }

    const presentedHash = crypto.createHash("sha256").update(presentedSecret).digest();
    const storedHash = Buffer.from(robot.linkSecretHash, "hex");
    if (presentedHash.length !== storedHash.length || !crypto.timingSafeEqual(presentedHash, storedHash)) {
      throw new UnauthorizedError("Invalid robot secret");
    }

    await robotRepository.markSeen(robot.robotId);
    req.robot = { id: robot._id.toString(), robotId: robot.robotId };
    next();
  });
}
