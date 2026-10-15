import { AuthRobot, AuthUser } from "./auth.types";

// Passport (@types/passport) declares `Express.Request.user?: Express.User`.
// Extending `Express.User` here (rather than redeclaring `Request.user`)
// lets our AuthUser fields flow through that existing declaration instead
// of conflicting with it.
declare global {
  namespace Express {
    // eslint-disable-next-line @typescript-eslint/no-empty-interface
    interface User extends AuthUser {}

    interface Request {
      // Set by requireRobotAuth for robot-facing service-to-service routes.
      // Distinct from `user` (human sessions) so the two auth domains can
      // never be confused by a handler checking the wrong field.
      robot?: AuthRobot;
    }
  }
}

export {};
