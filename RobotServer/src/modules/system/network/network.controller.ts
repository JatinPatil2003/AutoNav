import { Request, Response } from "express";
import { NetworkService } from "./network.service";

export class NetworkController {
  constructor(private readonly networkService: NetworkService) {}

  listWifiNetworks = async (_req: Request, res: Response): Promise<void> => {
    const networks = await this.networkService.listWifiNetworks();
    res.status(200).json({ networks });
  };

  getCurrentWifi = async (_req: Request, res: Response): Promise<void> => {
    const current = await this.networkService.getCurrentWifi();
    res.status(200).json(current);
  };

  connectWifi = async (req: Request, res: Response): Promise<void> => {
    const result = await this.networkService.connectWifi(req.body);
    res.status(result.success ? 200 : 502).json(result);
  };

  controlVpn = async (req: Request, res: Response): Promise<void> => {
    const result = await this.networkService.controlVpn(req.body);
    res.status(result.success ? 200 : 502).json(result);
  };
}
