import { IRobotDocument } from "./robot.model";

export interface RobotDto {
  id: string;
  name: string;
  displayName?: string;
  vpnHostname: string;
  status: string;
  assignedTo?: string;
}

export function toRobotDto(robot: IRobotDocument): RobotDto {
  return {
    id: robot._id.toString(),
    name: robot.name,
    displayName: robot.displayName,
    vpnHostname: robot.vpnHostname,
    status: robot.status,
    assignedTo: robot.assignedTo?.toString(),
  };
}
