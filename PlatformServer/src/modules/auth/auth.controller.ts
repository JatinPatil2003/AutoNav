import { Request, Response } from "express";
import { UnauthorizedError } from "../../common/errors";
import { clearAuthCookies, setAuthCookies } from "../../common/utils/cookie.util";
import { verifyRefreshToken } from "../../common/utils/jwt.util";
import { config } from "../../config/env";
import { GoogleProfile } from "../../config/passport";
import { AuthService } from "./auth.service";

export class AuthController {
  constructor(private readonly authService: AuthService) {}

  signup = async (req: Request, res: Response): Promise<void> => {
    const { user, tokens } = await this.authService.signup(req.body);
    setAuthCookies(res, tokens);
    res.status(201).json({ user });
  };

  login = async (req: Request, res: Response): Promise<void> => {
    const { user, tokens } = await this.authService.login(req.body);
    setAuthCookies(res, tokens);
    res.status(200).json({ user });
  };

  googleCallback = async (req: Request, res: Response): Promise<void> => {
    const profile = req.user as unknown as GoogleProfile;
    const { tokens } = await this.authService.handleGoogleLogin(profile);
    setAuthCookies(res, tokens);
    res.redirect(`${config.frontendUrl}/auth/callback`);
  };

  refresh = async (req: Request, res: Response): Promise<void> => {
    const refreshToken = req.cookies?.refreshToken as string | undefined;
    if (!refreshToken) {
      throw new UnauthorizedError("No refresh token provided");
    }

    const tokens = await this.authService.refresh(refreshToken);
    setAuthCookies(res, tokens);
    res.status(200).json({});
  };

  setPassword = async (req: Request, res: Response): Promise<void> => {
    const user = await this.authService.setPassword(req.user!.id, req.body);
    res.status(200).json({ user });
  };

  forgotPassword = async (req: Request, res: Response): Promise<void> => {
    await this.authService.forgotPassword(req.body);
    res.status(200).json({ message: "If that email exists, a reset code has been sent." });
  };

  verifyOtp = async (req: Request, res: Response): Promise<void> => {
    const result = await this.authService.verifyOtp(req.body);
    res.status(200).json(result);
  };

  resetPassword = async (req: Request, res: Response): Promise<void> => {
    await this.authService.resetPassword(req.body);
    res.status(200).json({ message: "Password reset successfully." });
  };

  logout = async (req: Request, res: Response): Promise<void> => {
    const refreshToken = req.cookies?.refreshToken as string | undefined;
    const userId = this.resolveUserIdFromRefreshToken(refreshToken);

    if (userId) {
      await this.authService.logout(userId);
    }

    clearAuthCookies(res);
    res.status(204).send();
  };

  private resolveUserIdFromRefreshToken(refreshToken: string | undefined): string | undefined {
    if (!refreshToken) return undefined;
    try {
      return verifyRefreshToken(refreshToken).sub;
    } catch {
      return undefined;
    }
  }
}
