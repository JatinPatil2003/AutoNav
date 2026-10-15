export interface AuthUser {
  userId: string;
  email: string;
  name: string;
  avatarUrl?: string;
}

export interface RobotSessionAccessPayload {
  sub: string;
  email: string;
  name: string;
  avatarUrl?: string;
  type: "robot-session-access";
}

export interface RobotSessionRefreshPayload {
  sub: string;
  email: string;
  name: string;
  avatarUrl?: string;
  type: "robot-session-refresh";
}

export interface RobotSessionTokenPair {
  accessToken: string;
  refreshToken: string;
}
