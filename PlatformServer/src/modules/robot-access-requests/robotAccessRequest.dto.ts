import { IRobotAccessRequestDocument } from "./robotAccessRequest.model";
import { IRobotDocument } from "../robots/robot.model";
import { PublicRobotDto } from "../users/user.dto";

export interface RobotAccessRequestDto {
  id: string;
  status: string;
  requestedBy?: { id: string; email: string; name: string };
  robot?: PublicRobotDto;
  reviewedAt?: string;
  rejectionReason?: string;
  createdAt: string;
}

export function toRobotAccessRequestDto(
  request: IRobotAccessRequestDocument,
): RobotAccessRequestDto {
  const populatedUser = request.user as unknown as
    | { _id: { toString(): string }; email: string; name: string }
    | undefined;
  const isUserPopulated = !!populatedUser && typeof populatedUser === "object" && "email" in populatedUser;

  const populatedRobot = request.robot as unknown as IRobotDocument | undefined;
  const isRobotPopulated = !!populatedRobot && typeof populatedRobot === "object" && "name" in populatedRobot;

  return {
    id: request._id.toString(),
    status: request.status,
    requestedBy: isUserPopulated
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
    reviewedAt: request.reviewedAt?.toISOString(),
    rejectionReason: request.rejectionReason,
    createdAt: request.createdAt.toISOString(),
  };
}
