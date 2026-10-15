import { NextFunction, Request, Response } from "express";
import { UnauthorizedError } from "../errors";
import { verifySessionAccessToken } from "../utils/jwt.util";
import { parseCookies } from "../utils/parseCookies.util";
import { AuthUser } from "../types/auth.types";

/**
 * Verifies a RobotApp session access token from a raw cookie header value.
 * Shared by requireAuth (Express middleware, has req.cookies) and the WS
 * upgrade path (core/wsRouter.ts, only has the raw header — no Express
 * middleware chain runs on a raw 'upgrade' event) so there is exactly one
 * place that decides what makes a session valid.
 */
export function verifyAccessTokenFromCookieHeader(cookieHeader: string | undefined): AuthUser | null {
  const token = parseCookies(cookieHeader).robotAccessToken;
  if (!token) return null;

  try {
    const payload = verifySessionAccessToken(token);
    return { userId: payload.sub, email: payload.email, name: payload.name, avatarUrl: payload.avatarUrl };
  } catch {
    return null;
  }
}

export function requireAuth(req: Request, _res: Response, next: NextFunction): void {
  const token = req.cookies?.robotAccessToken as string | undefined;
  if (!token) {
    throw new UnauthorizedError("Authentication required");
  }

  try {
    const payload = verifySessionAccessToken(token);
    req.user = { userId: payload.sub, email: payload.email, name: payload.name, avatarUrl: payload.avatarUrl };
    next();
  } catch {
    throw new UnauthorizedError("Invalid or expired access token");
  }
}
