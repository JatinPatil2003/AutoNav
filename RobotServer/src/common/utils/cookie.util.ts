import { Response } from "express";
import { config } from "../../config/env";
import { RobotSessionTokenPair } from "../types/auth.types";

// RobotApp is always served same-origin by this same RobotServer process
// (no cross-subdomain dashboard scenario like PlatformServer's cookies have
// to handle), so sameSite can be the strictest setting.
const baseCookieOptions = {
  httpOnly: true,
  secure: config.isProduction,
  sameSite: "strict" as const,
};

export function setSessionCookies(res: Response, tokens: RobotSessionTokenPair): void {
  res.cookie("robotAccessToken", tokens.accessToken, { ...baseCookieOptions, path: "/" });
  res.cookie("robotRefreshToken", tokens.refreshToken, {
    ...baseCookieOptions,
    path: "/api/hmi-auth",
  });
}

export function clearSessionCookies(res: Response): void {
  res.clearCookie("robotAccessToken", { ...baseCookieOptions, path: "/" });
  res.clearCookie("robotRefreshToken", { ...baseCookieOptions, path: "/api/hmi-auth" });
}
