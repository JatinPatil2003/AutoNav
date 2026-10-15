import { NextFunction, Request, Response } from "express";
import { NotFoundError, UnauthorizedError } from "../errors";
import { IRobotRepository } from "../../modules/robots/robot.repository.interface";
import { asyncHandler } from "../utils/asyncHandler.util";

/**
 * Identifies the calling robot for /api/robot-auth/* — no secret, the VPN
 * is the trust boundary (see root README). The robot sends only its own
 * device id (resolved on its side from the ESP32 bridge, see RobotServer's
 * ros/deviceIdentity.ts); this just resolves that id to a Robot record so
 * downstream handlers can scope data to it.
 */
export function createResolveRobot(robotRepository: IRobotRepository) {
  return asyncHandler(async (req: Request, _res: Response, next: NextFunction): Promise<void> => {
    const robotId = req.header("x-robot-id");
    if (!robotId) {
      throw new UnauthorizedError("x-robot-id header is required");
    }

    const robot = await robotRepository.findByRobotId(robotId);
    if (!robot) {
      throw new NotFoundError("Unknown robot");
    }

    await robotRepository.markSeen(robot.robotId);
    req.robot = { id: robot._id.toString(), robotId: robot.robotId };
    next();
  });
}
