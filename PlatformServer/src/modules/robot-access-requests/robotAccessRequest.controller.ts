import { Request, Response } from "express";
import { UnauthorizedError } from "../../common/errors";
import { RobotAccessRequestService } from "./robotAccessRequest.service";

export class RobotAccessRequestController {
  constructor(private readonly requestService: RobotAccessRequestService) {}

  createRequest = async (req: Request, res: Response): Promise<void> => {
    if (!req.user) throw new UnauthorizedError();
    const { robotId } = req.body;
    const request = await this.requestService.createRequest(req.user.id, robotId);
    res.status(201).json({ request });
  };

  listMine = async (req: Request, res: Response): Promise<void> => {
    if (!req.user) throw new UnauthorizedError();
    const requests = await this.requestService.listMine(req.user.id);
    res.status(200).json({ requests });
  };
}
