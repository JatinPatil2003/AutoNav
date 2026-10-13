import { IUserDocument } from "./user.model";
import { IRobotDocument } from "../robots/robot.model";

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
  robot: PublicRobotDto | null;
}

export function toPublicUserDto(user: IUserDocument): PublicUserDto {
  const robot = user.robot as unknown as IRobotDocument | undefined;
  const isPopulatedRobot = !!robot && typeof robot === "object" && "name" in robot;

  return {
    id: user._id.toString(),
    email: user.email,
    name: user.name,
    avatarUrl: user.avatarUrl,
    role: user.role,
    authProviders: user.authProviders,
    robot: isPopulatedRobot
      ? {
          id: robot._id.toString(),
          name: robot.name,
          displayName: robot.displayName,
          vpnHostname: robot.vpnHostname,
          status: robot.status,
        }
      : null,
  };
}
