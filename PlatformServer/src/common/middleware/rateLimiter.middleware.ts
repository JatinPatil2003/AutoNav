import rateLimit from "express-rate-limit";

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
