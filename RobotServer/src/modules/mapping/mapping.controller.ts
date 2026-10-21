import { Request, Response } from "express";
import { MappingService } from "./mapping.service";

export class MappingController {
  constructor(private readonly mappingService: MappingService) {}

  start = async (req: Request, res: Response): Promise<void> => {
    await this.mappingService.start(req.body.backend);
    res.status(200).json({ status: "started" });
  };

  stop = async (_req: Request, res: Response): Promise<void> => {
    this.mappingService.stop();
    res.status(200).json({ status: "stopped" });
  };

  getCurrentMap = async (_req: Request, res: Response): Promise<void> => {
    res.status(200).json(this.mappingService.getCurrentMap());
  };

  getCurrentLocation = async (_req: Request, res: Response): Promise<void> => {
    res.status(200).json(this.mappingService.getCurrentLocation());
  };

  captureWaypoint = async (req: Request, res: Response): Promise<void> => {
    await this.mappingService.captureWaypoint(req.body.name);
    res.status(201).json({ status: "captured" });
  };

  saveMap = async (req: Request, res: Response): Promise<void> => {
    await this.mappingService.saveMap(req.body.mapName);
    res.status(200).json({ status: "saved" });
  };

  commitWaypoints = async (req: Request, res: Response): Promise<void> => {
    const processed = await this.mappingService.commitWaypoints(req.body.mapName);
    res.status(200).json({ status: "committed", processed });
  };
}
