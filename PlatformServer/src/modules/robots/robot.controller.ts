import { Request, Response } from "express";
import { RobotService } from "./robot.service";
import { RobotStatus } from "./robot.model";

export class RobotController {
  constructor(private readonly robotService: RobotService) {}

  createRobot = async (req: Request, res: Response): Promise<void> => {
    const robot = await this.robotService.createRobot(req.body);
    res.status(201).json({ robot });
  };

  listRobots = async (req: Request, res: Response): Promise<void> => {
    const status = req.query.status as RobotStatus | undefined;
    const robots = status ? await this.robotService.listByStatus(status) : await this.robotService.listAll();
    res.status(200).json({ robots });
  };
}
