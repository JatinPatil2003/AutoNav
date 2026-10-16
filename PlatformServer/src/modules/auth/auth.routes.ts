import { Router } from "express";
import { passport } from "../../config/passport";
import { authRateLimiter, otpRateLimiter } from "../../common/middleware/rateLimiter.middleware";
import { requireAuth } from "../../common/middleware/auth.middleware";
import { validate } from "../../common/middleware/validate.middleware";
import { asyncHandler } from "../../common/utils/asyncHandler.util";
import { AuthController } from "./auth.controller";
import {
  forgotPasswordSchema,
  googleTokenSchema,
  loginSchema,
  resetPasswordSchema,
  setPasswordSchema,
  signupSchema,
  verifyOtpSchema,
} from "./auth.validation";

export function createAuthRouter(authController: AuthController): Router {
  const router = Router();

  router.use(authRateLimiter);

  /**
   * @openapi
   * /api/auth/signup:
   *   post:
   *     summary: Create an account with email and password
   *     tags: [Auth]
   *     responses:
   *       201:
   *         description: Account created, auth cookies set
   *       409:
   *         description: Email already registered
   */
  router.post("/signup", validate({ body: signupSchema }), asyncHandler(authController.signup));

  /**
   * @openapi
   * /api/auth/login:
   *   post:
   *     summary: Log in with email and password
   *     tags: [Auth]
   *     responses:
   *       200:
   *         description: Auth cookies set
   *       401:
   *         description: Invalid credentials
   */
  router.post("/login", validate({ body: loginSchema }), asyncHandler(authController.login));

  /**
   * @openapi
   * /api/auth/google:
   *   get:
   *     summary: Start Google OAuth sign-in
   *     tags: [Auth]
   *     responses:
   *       302:
   *         description: Redirects to Google's consent screen
   */
  router.get(
    "/google",
    passport.authenticate("google", { scope: ["profile", "email"], session: false }),
  );

  /**
   * @openapi
   * /api/auth/google/callback:
   *   get:
   *     summary: Google OAuth callback — issues auth cookies and redirects to the frontend
   *     tags: [Auth]
   *     responses:
   *       302:
   *         description: Redirects to the frontend's /auth/callback route
   */
  router.get(
    "/google/callback",
    passport.authenticate("google", { session: false, failureRedirect: "/api/auth/google/failure" }),
    asyncHandler(authController.googleCallback),
  );

  router.get("/google/failure", (_req, res) => {
    res.status(401).json({ error: "Unauthorized", message: "Google authentication failed" });
  });

  /**
   * @openapi
   * /api/auth/google/token:
   *   post:
   *     summary: Native-client Google sign-in — exchange a Google ID token (from the device's own Google Sign-In SDK) for an AutoNav session
   *     tags: [Auth]
   *     responses:
   *       200:
   *         description: Auth cookies set; mobile clients (header X-Client=mobile) also get the token pair in the body
   *       401:
   *         description: Invalid or unverifiable Google ID token
   */
  router.post(
    "/google/token",
    validate({ body: googleTokenSchema }),
    asyncHandler(authController.googleToken),
  );

  /**
   * @openapi
   * /api/auth/refresh:
   *   post:
   *     summary: Exchange a valid refresh token cookie for a new token pair
   *     tags: [Auth]
   *     responses:
   *       200:
   *         description: New auth cookies set
   *       401:
   *         description: Missing, invalid, or revoked refresh token
   */
  router.post("/refresh", asyncHandler(authController.refresh));

  /**
   * @openapi
   * /api/auth/logout:
   *   post:
   *     summary: Log out, revoking all outstanding refresh tokens for the user
   *     tags: [Auth]
   *     responses:
   *       204:
   *         description: Logged out, cookies cleared
   */
  router.post("/logout", asyncHandler(authController.logout));

  /**
   * @openapi
   * /api/auth/set-password:
   *   post:
   *     summary: Add/replace a local password on the authenticated account (e.g. for a Google-only signup)
   *     tags: [Auth]
   *     security:
   *       - cookieAuth: []
   *     responses:
   *       200:
   *         description: Password set; account can now also log in via email/password
   *       401:
   *         description: Not authenticated
   */
  router.post(
    "/set-password",
    requireAuth,
    validate({ body: setPasswordSchema }),
    asyncHandler(authController.setPassword),
  );

  /**
   * @openapi
   * /api/auth/forgot-password:
   *   post:
   *     summary: Request a password-reset OTP via email
   *     description: Always responds 200, whether or not the email is registered, to avoid account enumeration.
   *     tags: [Auth]
   *     responses:
   *       200:
   *         description: OTP sent if the account exists
   */
  router.post(
    "/forgot-password",
    otpRateLimiter,
    validate({ body: forgotPasswordSchema }),
    asyncHandler(authController.forgotPassword),
  );

  /**
   * @openapi
   * /api/auth/verify-otp:
   *   post:
   *     summary: Exchange a valid OTP for a short-lived password-reset token
   *     tags: [Auth]
   *     responses:
   *       200:
   *         description: Returns a resetToken to use with /auth/reset-password
   *       401:
   *         description: Invalid or expired code
   */
  router.post(
    "/verify-otp",
    otpRateLimiter,
    validate({ body: verifyOtpSchema }),
    asyncHandler(authController.verifyOtp),
  );

  /**
   * @openapi
   * /api/auth/reset-password:
   *   post:
   *     summary: Set a new password using a resetToken from /auth/verify-otp
   *     description: Revokes all existing refresh tokens, logging the account out everywhere.
   *     tags: [Auth]
   *     responses:
   *       200:
   *         description: Password reset
   *       401:
   *         description: Invalid or expired reset token
   */
  router.post(
    "/reset-password",
    otpRateLimiter,
    validate({ body: resetPasswordSchema }),
    asyncHandler(authController.resetPassword),
  );

  return router;
}
