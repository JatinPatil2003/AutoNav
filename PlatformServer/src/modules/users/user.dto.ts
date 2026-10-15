import { IUserDocument } from "./user.model";

export interface PublicRobotDto {
  id: string;
  name: string;
  displayName?: string;
  vpnHostname: string;
  status: string;
}

export interface PublicUserDto {
  id: string;
  email: string;
  name: string;
  avatarUrl?: string;
  role: string;
  authProviders: ("local" | "google")[];
  robots: PublicRobotDto[];
}

export function toPublicUserDto(user: IUserDocument, robots: PublicRobotDto[] = []): PublicUserDto {
  return {
    id: user._id.toString(),
    email: user.email,
    name: user.name,
    avatarUrl: user.avatarUrl,
    role: user.role,
    authProviders: user.authProviders,
    robots,
  };
}
