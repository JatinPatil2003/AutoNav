import { RobotAccessRequest } from "../robot-access/robotAccess.types";
import { CurrentUser } from "../auth/auth.types";

export type { RobotAccessRequest };
export type { CurrentUser as AdminUser };

export interface AdminRobot {
  id: string;
  robotId: string;
  name: string;
  displayName?: string;
  vpnHostname: string;
  status: string;
  lastSeenAt?: string;
}

export interface CreateRobotRequest {
  robotId: string;
  name: string;
  vpnHostname: string;
  displayName?: string;
}

/** The plaintext robotSecret is only ever present in this one response — never retrievable again. */
export interface CreateRobotResult {
  robot: AdminRobot;
  robotSecret: string;
}

export interface RobotAssignment {
  id: string;
  user?: { id: string; email: string; name: string };
  robot?: { id: string; name: string; displayName?: string; vpnHostname: string; status: string };
  status: string;
  /** Can log into this robot's own RobotApp kiosk (PIN login). */
  hmiAccess: boolean;
  /** Shows up in this user's "my robots" list in PlatformApp. */
  platformAccess: boolean;
  pinSet: boolean;
  grantedAt: string;
  revokedAt?: string;
  revokedReason?: string;
}
