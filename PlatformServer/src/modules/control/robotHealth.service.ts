import { NotFoundError } from "../../common/errors";
import { config } from "../../config/env";
import { IRobotRepository } from "../robots/robot.repository.interface";
import { IRobotConnectionRegistry } from "../robot-link/robotConnectionRegistry.interface";
import { RobotHealthDto } from "./robotHealth.dto";

/**
 * Backs `GET /api/robots/:robotId/health` — the literal "is this robot
 * reachable and sane" gate the client shows before letting a user drive
 * it. Deliberately reports only what's actually implemented: link
 * reachability (from the robot-link connection) and a one-shot ROS
 * liveness round trip. No battery/odometry fields — `system/diagnostics`
 * doesn't exist on RobotServer yet.
 */
export class RobotHealthService {
  constructor(
    private readonly robotRepository: IRobotRepository,
    private readonly registry: IRobotConnectionRegistry,
  ) {}

  async getHealth(robotMongoId: string): Promise<RobotHealthDto> {
    const robot = await this.robotRepository.findById(robotMongoId);
    if (!robot) throw new NotFoundError("Robot not found");

    const linkOk = this.registry.isOnline(robot.robotId);
    const rosResult = linkOk
      ? await this.registry.queryHealth(robot.robotId, config.healthQueryTimeoutMs)
      : null;

    return {
      robotId: robot._id.toString(),
      healthy: linkOk && !!rosResult?.rclOk,
      checkedAt: new Date().toISOString(),
      checks: {
        link: {
          ok: linkOk,
          connectedSince: this.registry.getConnectedSince(robot.robotId)?.toISOString(),
          rttMs: this.registry.getRttMs(robot.robotId),
        },
        ros: {
          ok: !!rosResult?.rclOk,
          navigationActive: rosResult?.navigationActive,
          maxLinear: rosResult?.maxLinear,
          maxAngular: rosResult?.maxAngular,
        },
      },
    };
  }
}
