export interface RobotSummary {
  id: string;
  name: string;
  displayName?: string;
  vpnHostname: string;
  status: string;
}

export type UserRole = "user" | "admin";

export interface CurrentUser {
  id: string;
  email: string;
  name: string;
  avatarUrl?: string;
  role: UserRole;
  authProviders: ("local" | "google")[];
  robots: RobotSummary[];
}

export interface SignupRequest {
  email: string;
  password: string;
  name: string;
}

export interface LoginRequest {
  email: string;
  password: string;
}
