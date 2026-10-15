import { IRobotAssignmentDocument } from "./robotAssignment.model";
import { IRobotDocument } from "../robots/robot.model";
import { PublicRobotDto } from "../users/user.dto";

export interface RobotAssignmentDto {
  id: string;
  user?: { id: string; email: string; name: string };
  robot?: PublicRobotDto;
  status: string;
  hmiAccess: boolean;
  platformAccess: boolean;
  pinSet: boolean;
  grantedAt: string;
  revokedAt?: string;
  revokedReason?: string;
}

export function toRobotAssignmentDto(assignment: IRobotAssignmentDocument): RobotAssignmentDto {
  const populatedUser = assignment.user as unknown as
    | { _id: { toString(): string }; email: string; name: string }
    | undefined;
  const isUserPopulated = !!populatedUser && typeof populatedUser === "object" && "email" in populatedUser;

  const populatedRobot = assignment.robot as unknown as IRobotDocument | undefined;
  const isRobotPopulated = !!populatedRobot && typeof populatedRobot === "object" && "name" in populatedRobot;

  return {
    id: assignment._id.toString(),
    user: isUserPopulated
      ? { id: populatedUser!._id.toString(), email: populatedUser!.email, name: populatedUser!.name }
      : undefined,
    robot: isRobotPopulated
      ? {
          id: populatedRobot!._id.toString(),
          name: populatedRobot!.name,
          displayName: populatedRobot!.displayName,
          vpnHostname: populatedRobot!.vpnHostname,
          status: populatedRobot!.status,
        }
      : undefined,
    status: assignment.status,
    hmiAccess: assignment.hmiAccess,
    platformAccess: assignment.platformAccess,
    pinSet: !!assignment.pinSetAt,
    grantedAt: assignment.grantedAt.toISOString(),
    revokedAt: assignment.revokedAt?.toISOString(),
    revokedReason: assignment.revokedReason,
  };
}
