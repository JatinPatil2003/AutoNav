import { Request, Response } from "express";
import { NavigationService } from "./navigation.service";

export class NavigationController {
  constructor(private readonly navigationService: NavigationService) {}

  start = async (req: Request, res: Response): Promise<void> => {
    await this.navigationService.start(req.body.mapName);
    res.status(200).json({ status: "started" });
  };

  stop = async (_req: Request, res: Response): Promise<void> => {
    this.navigationService.stop();
    res.status(200).json({ status: "stopped" });
  };

  listMaps = async (_req: Request, res: Response): Promise<void> => {
    res.status(200).json({ maps: await this.navigationService.listMaps() });
  };

  selectMap = async (req: Request, res: Response): Promise<void> => {
    await this.navigationService.selectMap(req.params.mapName);
    res.status(200).json({ status: "selected" });
  };

  listGoals = async (req: Request, res: Response): Promise<void> => {
    res.status(200).json(await this.navigationService.listGoals(req.params.mapName));
  };

  saveGoal = async (req: Request, res: Response): Promise<void> => {
    const { mapName, name, ...pose } = req.body;
    await this.navigationService.saveGoal(mapName, name, pose);
    res.status(201).json({ status: "saved" });
  };

  getGoal = async (req: Request, res: Response): Promise<void> => {
    const { mapName, name } = req.body;
    res.status(200).json(await this.navigationService.getGoal(mapName, name));
  };

  localize = async (req: Request, res: Response): Promise<void> => {
    const { mapName, name } = req.body;
    res.status(200).json(await this.navigationService.localize(mapName, name));
  };

  startGoal = async (req: Request, res: Response): Promise<void> => {
    const { mapName, name } = req.body;
    await this.navigationService.startGoal(mapName, name);
    res.status(200).json({ status: "navigating" });
  };

  cancelGoal = async (_req: Request, res: Response): Promise<void> => {
    await this.navigationService.cancelGoal();
    res.status(200).json({ status: "cancelled" });
  };

  getFeedback = async (_req: Request, res: Response): Promise<void> => {
    res.status(200).json({ feedback: this.navigationService.getFeedback() });
  };

  getStatus = async (_req: Request, res: Response): Promise<void> => {
    res.status(200).json({ status: this.navigationService.getStatus() });
  };

  setInitialPose = async (req: Request, res: Response): Promise<void> => {
    await this.navigationService.setInitialPose(req.body);
    res.status(200).json({ status: "set" });
  };

  getCurrentMap = async (_req: Request, res: Response): Promise<void> => {
    res.status(200).json(this.navigationService.getCurrentMap());
  };

  getCurrentLocation = async (_req: Request, res: Response): Promise<void> => {
    res.status(200).json(this.navigationService.getCurrentLocation());
  };
}
