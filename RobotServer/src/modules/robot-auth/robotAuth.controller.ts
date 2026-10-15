import { Request, Response } from "express";
import { UnauthorizedError } from "../../common/errors";
import { clearSessionCookies, setSessionCookies } from "../../common/utils/cookie.util";
import { RobotAuthService } from "./robotAuth.service";

export class RobotAuthController {
  constructor(private readonly robotAuthService: RobotAuthService) {}

  listUsers = async (_req: Request, res: Response): Promise<void> => {
    const users = await this.robotAuthService.listActiveUsers();
    res.status(200).json({ users });
  };

  identify = async (req: Request, res: Response): Promise<void> => {
    const { email } = req.body;
    const result = await this.robotAuthService.identify(email);
    res.status(200).json(result);
  };

  login = async (req: Request, res: Response): Promise<void> => {
    const { email, pin } = req.body;
    const tokens = await this.robotAuthService.loginWithPin(email, pin);
    setSessionCookies(res, tokens);
    res.status(200).json({ email });
  };

  refresh = async (req: Request, res: Response): Promise<void> => {
    const refreshToken = req.cookies?.robotRefreshToken as string | undefined;
    if (!refreshToken) {
      throw new UnauthorizedError("No refresh token provided");
    }
    const tokens = await this.robotAuthService.refresh(refreshToken);
    setSessionCookies(res, tokens);
    res.status(200).json({});
  };

  logout = async (_req: Request, res: Response): Promise<void> => {
    clearSessionCookies(res);
    res.status(204).send();
  };

  session = async (req: Request, res: Response): Promise<void> => {
    if (!req.user) throw new UnauthorizedError();
    res.status(200).json({ user: req.user });
  };

  requestPinSetupOtp = async (req: Request, res: Response): Promise<void> => {
    const { email } = req.body;
    await this.robotAuthService.requestPinSetupOtp(email);
    res.status(200).json({ message: "If that user has access to this robot, a code has been sent." });
  };

  verifyPinSetupOtp = async (req: Request, res: Response): Promise<void> => {
    const { email, otp } = req.body;
    const result = await this.robotAuthService.verifyPinSetupOtp(email, otp);
    res.status(200).json(result);
  };

  confirmPinSetup = async (req: Request, res: Response): Promise<void> => {
    const { pinSetupToken, pin } = req.body;
    const result = await this.robotAuthService.confirmPinSetup(pinSetupToken, pin);
    res.status(200).json(result);
  };
}
