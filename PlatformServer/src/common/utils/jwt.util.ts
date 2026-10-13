import jwt, { SignOptions } from "jsonwebtoken";
import { config } from "../../config/env";
import { AccessTokenPayload, RefreshTokenPayload, ResetTokenPayload } from "../types/auth.types";

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
