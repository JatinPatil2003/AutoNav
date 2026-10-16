import { Request, Response } from "express";
import { RobotService } from "./robot.service";
import { RobotStatus } from "./robot.model";

export class RobotController {
  constructor(private readonly robotService: RobotService) {}

  createRobot = async (req: Request, res: Response): Promise<void> => {
    const { robot, linkSecret } = await this.robotService.createRobot(req.body);
    res.status(201).json({ robot, linkSecret });
  };

  rotateLinkSecret = async (req: Request, res: Response): Promise<void> => {
    const { id } = req.params;
    const { linkSecret } = await this.robotService.rotateLinkSecret(id);
    res.status(200).json({ linkSecret });
  };

  listRobots = async (req: Request, res: Response): Promise<void> => {
    const status = req.query.status as RobotStatus | undefined;
    const robots = status ? await this.robotService.listByStatus(status) : await this.robotService.listAll();
    res.status(200).json({ robots });
  };

  listRobotSummaries = async (_req: Request, res: Response): Promise<void> => {
    const robots = await this.robotService.listAllSummaries();
    res.status(200).json({ robots });
  };

  deleteRobot = async (req: Request, res: Response): Promise<void> => {
    const { id } = req.params;
    await this.robotService.deleteRobot(id);
    res.status(204).send();
  };
}
