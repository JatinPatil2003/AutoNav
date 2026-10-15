import { Request, Response } from "express";
import { UnauthorizedError } from "../../common/errors";
import { RobotAuthService } from "./robotAuth.service";

export class RobotAuthController {
  constructor(private readonly robotAuthService: RobotAuthService) {}

  checkUserExists = async (req: Request, res: Response): Promise<void> => {
    const { email } = req.body;
    const exists = await this.robotAuthService.checkUserExists(email);
    res.status(200).json({ exists });
  };

  getRoster = async (req: Request, res: Response): Promise<void> => {
    if (!req.robot) throw new UnauthorizedError();
    const users = await this.robotAuthService.getRoster(req.robot.id);
    res.status(200).json({ users });
  };

  requestPinSetupOtp = async (req: Request, res: Response): Promise<void> => {
    if (!req.robot) throw new UnauthorizedError();
    const { email } = req.body;
    await this.robotAuthService.requestPinSetupOtp(req.robot.id, email);
    res.status(200).json({ message: "If that user has access to this robot, a code has been sent." });
  };

  verifyPinSetupOtp = async (req: Request, res: Response): Promise<void> => {
    if (!req.robot) throw new UnauthorizedError();
    const { email, otp } = req.body;
    const result = await this.robotAuthService.verifyPinSetupOtp(email, otp, req.robot.robotId);
    res.status(200).json(result);
  };

  confirmPinSetup = async (req: Request, res: Response): Promise<void> => {
    if (!req.robot) throw new UnauthorizedError();
    const { pinSetupToken, pin } = req.body;
    const entry = await this.robotAuthService.confirmPinSetup(
      pinSetupToken,
      pin,
      req.robot.robotId,
      req.robot.id,
    );
    res.status(200).json({ entry });
  };
}
