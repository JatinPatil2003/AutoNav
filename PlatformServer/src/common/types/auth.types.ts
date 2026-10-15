export type UserRole = "user" | "admin";

export interface AuthUser {
  id: string;
  role: UserRole;
}

export interface AccessTokenPayload {
  sub: string;
  role: UserRole;
}

export interface RefreshTokenPayload {
  sub: string;
  tokenVersion: number;
}

export interface TokenPair {
  accessToken: string;
  refreshToken: string;
}

export interface ResetTokenPayload {
  sub: string;
  purpose: "password-reset";
}

export interface PinSetupTokenPayload {
  sub: string;
  robotId: string;
  purpose: "robot-pin-setup";
}

export interface AuthRobot {
  id: string;
  robotId: string;
}
