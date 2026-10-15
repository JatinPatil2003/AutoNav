import { NextFunction, Request, Response } from "express";
import jwt from "jsonwebtoken";
import { UnauthorizedError } from "../errors";
import { config } from "../../config/env";

/**
 * Verifies an inbound platform->robot call (direction 2 of the
 * service-to-service JWT design, see PlatformServer's HttpRobotNotifier /
 * signPlatformToRobotToken). PlatformServer signs a fresh, 60s-TTL token
 * per call using *this robot's own* plaintext secret as the HMAC key — the
 * same value configured here as PLATFORM_ROBOT_SECRET — so verification is
 * purely symmetric, no round trip to PlatformServer needed. A leaked
 * secret for one robot can't be used to forge calls to any other robot.
 */
export function requirePlatformAuth(req: Request, _res: Response, next: NextFunction): void {
  const header = req.headers.authorization;
  if (!header?.startsWith("Bearer ")) {
    throw new UnauthorizedError("Platform authentication required");
  }

  const token = header.slice("Bearer ".length);

  try {
    // Verifying with this robot's own secret as the HMAC key already scopes
    // the token to this robot — only PlatformServer and this robot know it,
    // so a token signed for a different robot's secret fails verification
    // outright. No local robotId needed to additionally check `aud` against.
    const payload = jwt.verify(token, config.platform.robotSecret) as {
      iss?: string;
      purpose?: string;
    };
    if (payload.iss !== "platform-server" || payload.purpose !== "platform-to-robot") {
      throw new Error("Invalid token claims");
    }
    req.isPlatform = true;
    next();
  } catch {
    throw new UnauthorizedError("Invalid or expired platform token");
  }
}
