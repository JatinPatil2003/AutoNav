import { NextFunction, Request, Response } from "express";
import { UnauthorizedError } from "../errors";
import { decryptSecret, hashSecretForLookup, timingSafeEqual } from "../utils/secretCrypto.util";
import { IRobotRepository } from "../../modules/robots/robot.repository.interface";
import { asyncHandler } from "../utils/asyncHandler.util";

/**
 * Gates the robot-facing service-to-service endpoints (/api/robot-auth/*).
 * The robot sends only its provisioned `robotSecret` as a header — no
 * robotId needed, since the secret alone identifies the robot (looked up
 * by a SHA-256 fingerprint, see hashSecretForLookup). One less value to
 * provision/copy per robot, and the lookup is still O(1) via an index
 * instead of decrypting every robot's secret to find a match.
 */
export function createRequireRobotAuth(robotRepository: IRobotRepository) {
  return asyncHandler(async (req: Request, _res: Response, next: NextFunction): Promise<void> => {
    const robotSecret = req.header("x-robot-secret");
    if (!robotSecret) {
      throw new UnauthorizedError("Robot authentication required");
    }

    const robot = await robotRepository.findBySecretHash(hashSecretForLookup(robotSecret));
    if (!robot) {
      throw new UnauthorizedError("Invalid robot credentials");
    }

    // Belt-and-suspenders: the hash lookup already identifies the robot;
    // this guards against a hash collision being exploitable by also
    // requiring the decrypted plaintext to match exactly.
    const expectedSecret = decryptSecret(robot.robotSecretEncrypted);
    if (!timingSafeEqual(robotSecret, expectedSecret)) {
      throw new UnauthorizedError("Invalid robot credentials");
    }

    await robotRepository.markSeen(robot.robotId);
    req.robot = { id: robot._id.toString(), robotId: robot.robotId };
    next();
  });
}
