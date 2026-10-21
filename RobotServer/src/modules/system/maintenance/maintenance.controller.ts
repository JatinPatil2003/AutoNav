import { Request, Response } from "express";
import { MaintenanceService } from "./maintenance.service";

export class MaintenanceController {
  constructor(private readonly maintenanceService: MaintenanceService) {}

  reboot = async (_req: Request, res: Response): Promise<void> => {
    await this.maintenanceService.requestReboot();
    res.status(202).json({ status: "reboot_requested" });
  };

  rotateCredentials = async (_req: Request, res: Response): Promise<void> => {
    const result = await this.maintenanceService.rotateCredentials();
    res.status(200).json(result);
  };
}
