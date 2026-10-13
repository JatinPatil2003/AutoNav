import { Request, Response } from "express";
import { UnauthorizedError } from "../../common/errors";
import { RobotAccessRequestService } from "./robotAccessRequest.service";

export class RobotAccessRequestController {
  constructor(private readonly requestService: RobotAccessRequestService) {}

  createRequest = async (req: Request, res: Response): Promise<void> => {
    if (!req.user) throw new UnauthorizedError();
    const request = await this.requestService.createRequest(req.user.id);
    res.status(201).json({ request });
  };

  getMyLatestRequest = async (req: Request, res: Response): Promise<void> => {
    if (!req.user) throw new UnauthorizedError();
    const request = await this.requestService.getMyLatestRequest(req.user.id);
    res.status(200).json({ request });
  };
}
