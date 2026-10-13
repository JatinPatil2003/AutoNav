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
    const request = await this.adminService.approveRequest(id, robotId, req.user.id);
    res.status(200).json({ request });
  };

  rejectRequest = async (req: Request, res: Response): Promise<void> => {
    if (!req.user) throw new UnauthorizedError();
    const { id } = req.params;
    const { reason } = req.body;
    const request = await this.adminService.rejectRequest(id, req.user.id, reason);
    res.status(200).json({ request });
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
}
