# Changelog

All notable changes to `PlatformServer` are documented here. Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- Initial scaffold: layered Express/TypeScript backend (controller → service → repository interfaces).
- Auth: signup/login with email+password, Google OAuth2, JWT access/refresh tokens in httpOnly cookies, refresh-token revocation via `refreshTokenVersion`.
- User, Robot, and RobotAccessRequest Mongoose models.
- Robot-access-request workflow: user requests access, admin approves (assigning an unassigned robot) or rejects.
- First-admin bootstrap via `ADMIN_EMAILS`.
- Admin-only robot provisioning endpoint + `seedRobot.ts` script.
- OpenAPI docs generated from route JSDoc, served at `/api/docs`.
- Unit tests (in-memory fake repositories) and integration tests (`mongodb-memory-server` + supertest) for the auth and robot-access-request flows.
- `POST /api/auth/set-password`: lets an authenticated Google-only account add a local password (adds `'local'` to `authProviders` without disturbing the Google linkage). `PublicUserDto` now exposes `authProviders`.
- Admin user management: `GET /api/admin/users`, `PATCH /api/admin/users/:id/role` (promote/demote; an admin cannot change their own role).
- Admin robot management: `GET /api/admin/robots` with no `status` query now returns all robots (not just unassigned), for a full robots-overview view.
- Access logging: `pino-http` now logs the authenticated `userId` on every request and emits a one-line `METHOD url -> status` message; `logger.debug()` calls added through the auth and admin service flows (signup/login/logout, approve/reject, role changes) for traceability in development (`NODE_ENV=development` → debug level + pretty console output; silent in tests).
- Forgot-password OTP flow: `POST /api/auth/forgot-password` (always 200, prevents email enumeration), `POST /api/auth/verify-otp` (exchanges a valid 6-digit code for a short-lived, single-purpose reset token), `POST /api/auth/reset-password` (consumes the reset token, sets the new password, and revokes every outstanding refresh token). OTPs are bcrypt-hashed at rest in a new `PasswordResetOtp` collection with a TTL index (auto-expires) and are single-use. Email delivery is free (Gmail SMTP via Nodemailer) behind an `IEmailOtpSender` interface, so a paid provider (SES/Resend) can be swapped in later without touching `AuthService`. All three endpoints sit behind a dedicated `otpRateLimiter` (5/hour) on top of the existing `authRateLimiter`.
- Access log payload trimmed: `pino-http`'s default req/res serializers (which dumped full headers on every request) are now overridden to log just `{method, url}`/`{statusCode}`, so the one-line `customSuccessMessage` is what actually shows up in the console.
- Production log rotation: `logger.util.ts` now writes to both stdout and a rotating file (`pino-roll`, daily or 20MB, last 14 files kept) in production; dev keeps the plain `pino-pretty` console output unchanged. See `AWS/README.md` for the compression/retention cron job that runs on the EC2 host.
