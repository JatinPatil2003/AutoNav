import { Request, Response } from "express";
import { DiagnosticsService } from "./diagnostics.service";

export class DiagnosticsController {
  constructor(private readonly diagnosticsService: DiagnosticsService) {}

  getSensorHealth = async (req: Request, res: Response): Promise<void> => {
    res.status(200).json(this.diagnosticsService.getSensorHealth(req.params.sensorName));
  };

  getStatus = async (_req: Request, res: Response): Promise<void> => {
    res.status(200).json(this.diagnosticsService.getStatus());
  };

  getVersion = async (_req: Request, res: Response): Promise<void> => {
    res.status(200).json(this.diagnosticsService.getVersion());
  };

  getBatteryStatus = async (_req: Request, res: Response): Promise<void> => {
    res.status(200).json(await this.diagnosticsService.getBatteryStatus());
  };
}
