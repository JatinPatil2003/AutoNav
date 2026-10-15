import { AuthUser } from "./auth.types";

// req.user — a human logged into RobotApp (the local HMI PWA) via PIN.
// Calls from PlatformServer (/api/platform-auth/*) and to it carry no auth
// of their own — the VPN is the trust boundary (see root README).
declare global {
  namespace Express {
    interface Request {
      user?: AuthUser;
    }
  }
}

export {};
