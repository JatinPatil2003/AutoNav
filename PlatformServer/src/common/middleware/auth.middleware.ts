import { NextFunction, Request, Response } from "express";
import { UnauthorizedError } from "../errors";
import { verifyAccessToken } from "../utils/jwt.util";

export function requireAuth(req: Request, _res: Response, next: NextFunction): void {
  const token = req.cookies?.accessToken as string | undefined;

  if (!token) {
    throw new UnauthorizedError("Authentication required");
  }

  try {
    const payload = verifyAccessToken(token);
    req.user = { id: payload.sub, role: payload.role };
    next();
  } catch {
    throw new UnauthorizedError("Invalid or expired access token");
  }
}
