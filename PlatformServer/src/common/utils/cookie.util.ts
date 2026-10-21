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

/**
 * Raw WS upgrade requests never pass through Express's middleware chain
 * (the http.Server 'upgrade' event bypasses it entirely), so cookie-parser
 * never runs and `req.cookies` is undefined. Every WS gateway that wants
 * cookie-based auth parity with the dashboard's normal HTTP session
 * (ControlSessionGateway, TelemetryRelayGateway) reparses the Cookie header
 * by hand with this instead of duplicating the logic.
 */
export function parseCookieHeader(header: string | undefined): Record<string, string> {
  if (!header) return {};
  return Object.fromEntries(
    header
      .split(";")
      .map((pair) => pair.trim())
      .filter(Boolean)
      .map((pair) => {
        const index = pair.indexOf("=");
        if (index === -1) return [pair, ""];
        return [decodeURIComponent(pair.slice(0, index)), decodeURIComponent(pair.slice(index + 1))];
      }),
  );
}
