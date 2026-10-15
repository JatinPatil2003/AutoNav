import { IRobotDocument } from "./robot.model";

export interface RobotDto {
  id: string;
  robotId: string;
  name: string;
  displayName?: string;
  vpnHostname: string;
  status: string;
  lastSeenAt?: string;
}

/** Minimal shape exposed to non-admin users (e.g. the request-access picker). */
export interface RobotSummaryDto {
  id: string;
  robotId: string;
  name: string;
  displayName?: string;
}

export function toRobotDto(robot: IRobotDocument): RobotDto {
  return {
    id: robot._id.toString(),
    robotId: robot.robotId,
    name: robot.name,
    displayName: robot.displayName,
    vpnHostname: robot.vpnHostname,
    status: robot.status,
    lastSeenAt: robot.lastSeenAt?.toISOString(),
  };
}

export function toRobotSummaryDto(robot: IRobotDocument): RobotSummaryDto {
  return {
    id: robot._id.toString(),
    robotId: robot.robotId,
    name: robot.name,
    displayName: robot.displayName,
  };
}
