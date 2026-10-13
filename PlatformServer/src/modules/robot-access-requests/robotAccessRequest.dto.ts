import { IRobotAccessRequestDocument } from "./robotAccessRequest.model";

export interface RobotAccessRequestDto {
  id: string;
  status: string;
  requestedBy?: { id: string; email: string; name: string };
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
  const isPopulated = !!populatedUser && typeof populatedUser === "object" && "email" in populatedUser;

  return {
    id: request._id.toString(),
    status: request.status,
    requestedBy: isPopulated
      ? { id: populatedUser!._id.toString(), email: populatedUser!.email, name: populatedUser!.name }
      : undefined,
    reviewedAt: request.reviewedAt?.toISOString(),
    rejectionReason: request.rejectionReason,
    createdAt: request.createdAt.toISOString(),
  };
}
