import { Response } from "express";
import { config } from "../../config/env";
import { TokenPair } from "../types/auth.types";

const baseCookieOptions = {
  httpOnly: true,
  secure: config.isProduction,
  sameSite: "lax" as const,
  domain: config.isProduction ? config.cookieDomain : undefined,
};

export function setAuthCookies(res: Response, tokens: TokenPair): void {
  res.cookie("accessToken", tokens.accessToken, {
    ...baseCookieOptions,
    path: "/",
  });
  res.cookie("refreshToken", tokens.refreshToken, {
    ...baseCookieOptions,
    path: "/api/auth",
  });
}

export function clearAuthCookies(res: Response): void {
  res.clearCookie("accessToken", { ...baseCookieOptions, path: "/" });
  res.clearCookie("refreshToken", { ...baseCookieOptions, path: "/api/auth" });
}
