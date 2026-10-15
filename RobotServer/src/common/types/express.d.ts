import { AuthUser } from "./auth.types";

// RobotServer's own trust domains (deliberately separate from each other):
//   req.user       — a human logged into RobotApp (the local HMI PWA) via PIN
//   req.isPlatform — this specific request was verified as coming from
//                    PlatformServer (requirePlatformAuth); never set together
//                    with req.user, and never trusted from the same cookie.
declare global {
  namespace Express {
    interface Request {
      user?: AuthUser;
      isPlatform?: boolean;
    }
  }
}

export {};
