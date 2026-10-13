import { AuthUser } from "./auth.types";

// Passport (@types/passport) declares `Express.Request.user?: Express.User`.
// Extending `Express.User` here (rather than redeclaring `Request.user`)
// lets our AuthUser fields flow through that existing declaration instead
// of conflicting with it.
declare global {
  namespace Express {
    // eslint-disable-next-line @typescript-eslint/no-empty-interface
    interface User extends AuthUser {}
  }
}

export {};
