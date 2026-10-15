import { Request, Response } from "express";
import { UnauthorizedError } from "../../common/errors";
import { RobotAccessRequestService } from "../robot-access-requests/robotAccessRequest.service";
import { AdminService } from "./admin.service";

export class AdminController {
  constructor(
    private readonly adminService: AdminService,
    private readonly requestService: RobotAccessRequestService,
  ) {}

  listRequests = async (req: Request, res: Response): Promise<void> => {
    const status = (req.query.status as "pending" | "approved" | "rejected" | undefined) ?? "pending";
    const requests = await this.requestService.listByStatus(status);
    res.status(200).json({ requests });
  };

  approveRequest = async (req: Request, res: Response): Promise<void> => {
    if (!req.user) throw new UnauthorizedError();
    const { id } = req.params;
    const { robotId } = req.body;
    const request = await this.adminService.approveRequest(id, req.user.id, robotId);
    res.status(200).json({ request });
  };

  rejectRequest = async (req: Request, res: Response): Promise<void> => {
    if (!req.user) throw new UnauthorizedError();
    const { id } = req.params;
    const { reason } = req.body;
    const request = await this.adminService.rejectRequest(id, req.user.id, reason);
    res.status(200).json({ request });
  };

  grantRobotAssignment = async (req: Request, res: Response): Promise<void> => {
    if (!req.user) throw new UnauthorizedError();
    const { userId, robotId, hmiAccess, platformAccess } = req.body;
    const assignment = await this.adminService.grantRobotAssignment(userId, robotId, req.user.id, {
      hmiAccess,
      platformAccess,
    });
    res.status(201).json({ assignment });
  };

  revokeRobotAssignment = async (req: Request, res: Response): Promise<void> => {
    if (!req.user) throw new UnauthorizedError();
    const { id } = req.params;
    const { reason } = req.body;
    const assignment = await this.adminService.revokeRobotAssignment(id, req.user.id, reason);
    res.status(200).json({ assignment });
  };

  updateRobotAssignmentAccess = async (req: Request, res: Response): Promise<void> => {
    if (!req.user) throw new UnauthorizedError();
    const { id } = req.params;
    const { hmiAccess, platformAccess } = req.body;
    const assignment = await this.adminService.updateRobotAssignmentAccess(id, {
      hmiAccess,
      platformAccess,
    });
    res.status(200).json({ assignment });
  };

  listAssignmentsForRobot = async (req: Request, res: Response): Promise<void> => {
    const { robotId } = req.params;
    const assignments = await this.adminService.listAssignmentsForRobot(robotId);
    res.status(200).json({ assignments });
  };

  listAssignmentsForUser = async (req: Request, res: Response): Promise<void> => {
    const { userId } = req.params;
    const assignments = await this.adminService.listAssignmentsForUser(userId);
    res.status(200).json({ assignments });
  };

  listUsers = async (_req: Request, res: Response): Promise<void> => {
    const users = await this.adminService.listUsers();
    res.status(200).json({ users });
  };

  updateUserRole = async (req: Request, res: Response): Promise<void> => {
    if (!req.user) throw new UnauthorizedError();
    const { id } = req.params;
    const { role } = req.body;
    const user = await this.adminService.updateUserRole(id, role, req.user.id);
    res.status(200).json({ user });
  };

  deleteUser = async (req: Request, res: Response): Promise<void> => {
    if (!req.user) throw new UnauthorizedError();
    const { id } = req.params;
    await this.adminService.deleteUser(id, req.user.id);
    res.status(204).send();
  };
}
