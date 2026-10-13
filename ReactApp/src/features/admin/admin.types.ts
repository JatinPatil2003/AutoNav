import { RobotAccessRequest } from "../robot-access/robotAccess.types";
import { CurrentUser } from "../auth/auth.types";

export type { RobotAccessRequest };
export type { CurrentUser as AdminUser };

export interface AdminRobot {
  id: string;
  name: string;
  displayName?: string;
  vpnHostname: string;
  status: string;
}

export interface CreateRobotRequest {
  name: string;
  vpnHostname: string;
  displayName?: string;
}
