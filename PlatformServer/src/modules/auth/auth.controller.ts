import { Request, Response } from "express";
import { UnauthorizedError } from "../../common/errors";
import { TokenPair } from "../../common/types/auth.types";
import { clearAuthCookies, setAuthCookies } from "../../common/utils/cookie.util";
import { verifyGoogleIdToken } from "../../common/utils/googleToken.util";
import { verifyRefreshToken } from "../../common/utils/jwt.util";
import { config } from "../../config/env";
import { GoogleProfile } from "../../config/passport";
import { PublicUserDto } from "../users/user.dto";
import { AuthService } from "./auth.service";

export class AuthController {
  constructor(private readonly authService: AuthService) {}

  signup = async (req: Request, res: Response): Promise<void> => {
    const { user, tokens } = await this.authService.signup(req.body);
    setAuthCookies(res, tokens);
    res.status(201).json(this.buildAuthResponse(req, user, tokens));
  };

  login = async (req: Request, res: Response): Promise<void> => {
    const { user, tokens } = await this.authService.login(req.body);
    setAuthCookies(res, tokens);
    res.status(200).json(this.buildAuthResponse(req, user, tokens));
  };

  googleCallback = async (req: Request, res: Response): Promise<void> => {
    const profile = req.user as unknown as GoogleProfile;
    const { tokens } = await this.authService.handleGoogleLogin(profile);
    setAuthCookies(res, tokens);
    res.redirect(`${config.frontendUrl}/auth/callback`);
  };

  /**
   * Native-client counterpart to the browser-redirect /google and
   * /google/callback flow above. The Android app exchanges an ID token from
   * the native google_sign_in SDK here instead of being bounced through a
   * WebView.
   */
  googleToken = async (req: Request, res: Response): Promise<void> => {
    const { idToken } = req.body as { idToken: string };

    let profile: GoogleProfile;
    try {
      profile = await verifyGoogleIdToken(idToken);
    } catch {
      throw new UnauthorizedError("Invalid Google ID token");
    }

    const { user, tokens } = await this.authService.handleGoogleLogin(profile);
    setAuthCookies(res, tokens);
    res.status(200).json(this.buildAuthResponse(req, user, tokens));
  };

  refresh = async (req: Request, res: Response): Promise<void> => {
    const refreshToken = this.extractRefreshToken(req);
    if (!refreshToken) {
      throw new UnauthorizedError("No refresh token provided");
    }

    const tokens = await this.authService.refresh(refreshToken);
    setAuthCookies(res, tokens);
    res.status(200).json(this.isMobileClient(req) ? { tokens } : {});
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
    const refreshToken = this.extractRefreshToken(req);
    const userId = this.resolveUserIdFromRefreshToken(refreshToken);

    if (userId) {
      await this.authService.logout(userId);
    }

    clearAuthCookies(res);
    res.status(204).send();
  };

  /**
   * Mobile has no meaningful cookie jar, so login/signup/google-token also
   * return the raw token pair in the body for a native client to persist
   * itself (e.g. flutter_secure_storage) and send back as a Bearer header.
   * Web callers never set this header and keep getting the cookie-only
   * shape they already expect.
   */
  private buildAuthResponse(
    req: Request,
    user: PublicUserDto,
    tokens: TokenPair,
  ): { user: PublicUserDto; tokens?: TokenPair } {
    return this.isMobileClient(req) ? { user, tokens } : { user };
  }

  private isMobileClient(req: Request): boolean {
    return req.header("X-Client")?.toLowerCase() === "mobile";
  }

  private extractRefreshToken(req: Request): string | undefined {
    return (
      (req.cookies?.refreshToken as string | undefined) ??
      (req.body?.refreshToken as string | undefined)
    );
  }

  private resolveUserIdFromRefreshToken(refreshToken: string | undefined): string | undefined {
    if (!refreshToken) return undefined;
    try {
      return verifyRefreshToken(refreshToken).sub;
    } catch {
      return undefined;
    }
  }
}
