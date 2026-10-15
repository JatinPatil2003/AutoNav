import jwt, { SignOptions } from "jsonwebtoken";
import { config } from "../../config/env";
import { RobotSessionAccessPayload, RobotSessionRefreshPayload } from "../types/auth.types";

export function signSessionAccessToken(payload: RobotSessionAccessPayload): string {
  return jwt.sign(payload, config.robotSession.accessSecret, {
    expiresIn: config.robotSession.accessTtl,
  } as SignOptions);
}

export function signSessionRefreshToken(payload: RobotSessionRefreshPayload): string {
  return jwt.sign(payload, config.robotSession.refreshSecret, {
    expiresIn: config.robotSession.refreshTtl,
  } as SignOptions);
}

export function verifySessionAccessToken(token: string): RobotSessionAccessPayload {
  const payload = jwt.verify(token, config.robotSession.accessSecret) as RobotSessionAccessPayload;
  if (payload.type !== "robot-session-access") {
    throw new Error("Invalid token type");
  }
  return payload;
}

export function verifySessionRefreshToken(token: string): RobotSessionRefreshPayload {
  const payload = jwt.verify(token, config.robotSession.refreshSecret) as RobotSessionRefreshPayload;
  if (payload.type !== "robot-session-refresh") {
    throw new Error("Invalid token type");
  }
  return payload;
}
