import rateLimit from "express-rate-limit";
import { Request } from "express";
import { hashSecretForLookup } from "../utils/secretCrypto.util";

export const authRateLimiter = rateLimit({
  windowMs: 15 * 60 * 1000,
  limit: 20,
  standardHeaders: true,
  legacyHeaders: false,
  message: {
    error: "TooManyRequests",
    message: "Too many authentication attempts. Please try again later.",
  },
});

/**
 * Tighter limit for the password-reset OTP flow — this is the highest
 * abuse-potential surface in the API (OTP brute-forcing, email-bombing a
 * victim's inbox), so it gets its own stricter budget on top of
 * authRateLimiter.
 */
export const otpRateLimiter = rateLimit({
  windowMs: 60 * 60 * 1000,
  limit: 5,
  standardHeaders: true,
  legacyHeaders: false,
  message: {
    error: "TooManyRequests",
    message: "Too many password reset attempts. Please try again later.",
  },
});

/**
 * Many robots share one VPN egress IP, so robot-facing limiters key by the
 * claimed secret's fingerprint (hashed, never the plaintext) instead of IP
 * — otherwise one noisy robot would exhaust the budget for every other
 * robot behind the same tunnel, and a brute-force attempt stays bounded
 * per claimed secret rather than resetting with each guess.
 */
function keyBySecretFingerprint(req: Request): string {
  const secret = req.header("x-robot-secret");
  return secret ? hashSecretForLookup(secret) : (req.ip ?? "unknown");
}

export const robotAuthRateLimiter = rateLimit({
  windowMs: 15 * 60 * 1000,
  limit: 60,
  standardHeaders: true,
  legacyHeaders: false,
  keyGenerator: keyBySecretFingerprint,
  message: {
    error: "TooManyRequests",
    message: "Too many robot authentication attempts. Please try again later.",
  },
});

export const robotOtpRateLimiter = rateLimit({
  windowMs: 60 * 60 * 1000,
  limit: 5,
  standardHeaders: true,
  legacyHeaders: false,
  keyGenerator: (req: Request) => `${req.robot?.robotId ?? "unknown"}:${req.body?.email ?? "unknown"}`,
  message: {
    error: "TooManyRequests",
    message: "Too many PIN setup attempts. Please try again later.",
  },
});
