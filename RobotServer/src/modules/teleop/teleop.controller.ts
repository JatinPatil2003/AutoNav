import { Request, Response } from "express";
import { TeleopService } from "./teleop.service";

export class TeleopController {
  constructor(private readonly teleopService: TeleopService) {}

  setVelocity = async (req: Request, res: Response): Promise<void> => {
    this.teleopService.setVelocity(req.body);
    res.status(200).json({ status: "set" });
  };

  setEmergency = async (req: Request, res: Response): Promise<void> => {
    this.teleopService.setEmergency(req.body.status);
    res.status(200).json({ status: req.body.status });
  };
}
