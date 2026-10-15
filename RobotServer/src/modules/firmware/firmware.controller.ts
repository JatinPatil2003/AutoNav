import { Request, Response } from "express";
import { FirmwareService } from "./firmware.service";

export class FirmwareController {
  constructor(private readonly firmwareService: FirmwareService) {}

  setLedPattern = async (req: Request, res: Response): Promise<void> => {
    const result = await this.firmwareService.setLedPattern(req.body);
    res.status(result.success ? 200 : 502).json(result);
  };

  setBrightness = async (req: Request, res: Response): Promise<void> => {
    const result = await this.firmwareService.setBrightness(req.body);
    res.status(result.success ? 200 : 502).json(result);
  };
}
