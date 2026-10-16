import { Request, Response } from "express";
import { ForbiddenError, UnauthorizedError } from "../../common/errors";
import { RobotAssignmentService } from "../robot-assignments/robotAssignment.service";
import { RobotHealthService } from "./robotHealth.service";

export class RobotHealthController {
  constructor(
    private readonly healthService: RobotHealthService,
    private readonly assignmentService: RobotAssignmentService,
  ) {}

  getHealth = async (req: Request, res: Response): Promise<void> => {
    if (!req.user) throw new UnauthorizedError();
    const { robotId } = req.params;

    const hasAccess = await this.assignmentService.hasActivePlatformAccess(req.user.id, robotId);
    if (!hasAccess) {
      throw new ForbiddenError("No active platform-access assignment for this robot");
    }

    const health = await this.healthService.getHealth(robotId);
    res.status(200).json(health);
  };
}
