/** Mirrors PlatformServer's src/modules/control/robotHealth.dto.ts exactly. */
export interface RobotHealthLinkCheck {
  ok: boolean;
  connectedSince?: string;
  rttMs?: number;
}

export interface RobotHealthRosCheck {
  ok: boolean;
  navigationActive?: boolean;
  /** This robot's real velocity envelope — scale joystick input onto this, not a guessed constant. */
  maxLinear?: number;
  maxAngular?: number;
}

export interface RobotHealth {
  robotId: string;
  healthy: boolean;
  checkedAt: string;
  checks: {
    link: RobotHealthLinkCheck;
    ros: RobotHealthRosCheck;
  };
}
