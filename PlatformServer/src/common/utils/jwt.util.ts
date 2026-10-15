import jwt, { SignOptions } from "jsonwebtoken";
import { config } from "../../config/env";
import {
  AccessTokenPayload,
  PinSetupTokenPayload,
  RefreshTokenPayload,
  ResetTokenPayload,
} from "../types/auth.types";

export function signAccessToken(payload: AccessTokenPayload): string {
  return jwt.sign(payload, config.jwt.accessSecret, {
    expiresIn: config.jwt.accessTtl,
  } as SignOptions);
}

export function signRefreshToken(payload: RefreshTokenPayload): string {
  return jwt.sign(payload, config.jwt.refreshSecret, {
    expiresIn: config.jwt.refreshTtl,
  } as SignOptions);
}

export function verifyAccessToken(token: string): AccessTokenPayload {
  return jwt.verify(token, config.jwt.accessSecret) as AccessTokenPayload;
}

export function verifyRefreshToken(token: string): RefreshTokenPayload {
  return jwt.verify(token, config.jwt.refreshSecret) as RefreshTokenPayload;
}

export function signResetToken(userId: string): string {
  const payload: ResetTokenPayload = { sub: userId, purpose: "password-reset" };
  return jwt.sign(payload, config.resetToken.secret, {
    expiresIn: config.resetToken.ttl,
  } as SignOptions);
}

export function verifyResetToken(token: string): ResetTokenPayload {
  const payload = jwt.verify(token, config.resetToken.secret) as ResetTokenPayload;
  if (payload.purpose !== "password-reset") {
    throw new Error("Invalid token purpose");
  }
  return payload;
}

export function signPinSetupToken(userId: string, robotId: string): string {
  const payload: PinSetupTokenPayload = { sub: userId, robotId, purpose: "robot-pin-setup" };
  return jwt.sign(payload, config.pinSetupToken.secret, {
    expiresIn: config.pinSetupToken.ttl,
  } as SignOptions);
}

export function verifyPinSetupToken(token: string): PinSetupTokenPayload {
  const payload = jwt.verify(token, config.pinSetupToken.secret) as PinSetupTokenPayload;
  if (payload.purpose !== "robot-pin-setup") {
    throw new Error("Invalid token purpose");
  }
  return payload;
}

/**
 * Signs a short-lived, single-use JWT for an outbound platform->robot call,
 * using that specific robot's own plaintext secret as the HMAC key (not a
 * Platform-global secret) — a leaked secret for one robot can't be used to
 * forge calls to any other robot. No refresh/session machinery is needed
 * since each call mints a fresh token.
 */
export function signPlatformToRobotToken(robotPlaintextSecret: string, robotId: string): string {
  return jwt.sign(
    { iss: "platform-server", aud: robotId, purpose: "platform-to-robot" },
    robotPlaintextSecret,
    { expiresIn: config.platformToRobotTokenTtl } as SignOptions,
  );
}
