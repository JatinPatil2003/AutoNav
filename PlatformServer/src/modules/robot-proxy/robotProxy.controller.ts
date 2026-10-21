import { Request, Response } from "express";
import { ForbiddenError, UnauthorizedError } from "../../common/errors";
import { RobotAssignmentService } from "../robot-assignments/robotAssignment.service";
import { RobotProxyService } from "./robotProxy.service";

/** Same gate as /ws/control and /api/robots/:robotId/health — platformAccess is what lets a user command this robot at all, not just hold an assignment. */
export class RobotProxyController {
  constructor(
    private readonly proxyService: RobotProxyService,
    private readonly assignmentService: RobotAssignmentService,
  ) {}

  private async requireAccess(req: Request): Promise<{ robotId: string }> {
    if (!req.user) throw new UnauthorizedError();
    const { robotId } = req.params;
    const hasAccess = await this.assignmentService.hasActivePlatformAccess(req.user.id, robotId);
    if (!hasAccess) throw new ForbiddenError("No active platform-access assignment for this robot");
    return { robotId };
  }

  startMapping = async (req: Request, res: Response): Promise<void> => {
    const { robotId } = await this.requireAccess(req);
    await this.proxyService.startMapping(robotId, req.body.backend);
    res.status(200).json({ status: "started" });
  };

  stopMapping = async (req: Request, res: Response): Promise<void> => {
    const { robotId } = await this.requireAccess(req);
    await this.proxyService.stopMapping(robotId);
    res.status(200).json({ status: "stopped" });
  };

  captureWaypoint = async (req: Request, res: Response): Promise<void> => {
    const { robotId } = await this.requireAccess(req);
    await this.proxyService.captureWaypoint(robotId, req.body.name);
    res.status(201).json({ status: "captured" });
  };

  commitWaypoints = async (req: Request, res: Response): Promise<void> => {
    const { robotId } = await this.requireAccess(req);
    const result = await this.proxyService.commitWaypoints(robotId, req.body.mapName);
    res.status(200).json({ status: "committed", ...result });
  };

  saveMap = async (req: Request, res: Response): Promise<void> => {
    const { robotId } = await this.requireAccess(req);
    await this.proxyService.saveMap(robotId, req.body.mapName);
    res.status(200).json({ status: "saved" });
  };

  listMaps = async (req: Request, res: Response): Promise<void> => {
    const { robotId } = await this.requireAccess(req);
    const result = await this.proxyService.listMaps(robotId);
    res.status(200).json(result);
  };

  selectMap = async (req: Request, res: Response): Promise<void> => {
    const { robotId } = await this.requireAccess(req);
    await this.proxyService.selectMap(robotId, req.params.mapName);
    res.status(200).json({ status: "selected" });
  };

  listGoals = async (req: Request, res: Response): Promise<void> => {
    const { robotId } = await this.requireAccess(req);
    const goals = await this.proxyService.listGoals(robotId, req.params.mapName);
    res.status(200).json(goals);
  };

  saveGoal = async (req: Request, res: Response): Promise<void> => {
    const { robotId } = await this.requireAccess(req);
    await this.proxyService.saveGoal(robotId, req.body);
    res.status(201).json({ status: "saved" });
  };

  startGoal = async (req: Request, res: Response): Promise<void> => {
    const { robotId } = await this.requireAccess(req);
    await this.proxyService.startGoal(robotId, req.body.mapName, req.body.name);
    res.status(200).json({ status: "started" });
  };

  cancelGoal = async (req: Request, res: Response): Promise<void> => {
    const { robotId } = await this.requireAccess(req);
    await this.proxyService.cancelGoal(robotId);
    res.status(200).json({ status: "canceled" });
  };
}
