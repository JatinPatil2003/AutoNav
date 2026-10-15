import { IRobotAssignmentDocument } from "../robot-assignments/robotAssignment.model";

interface MinimalUser {
  _id: { toString(): string };
  email: string;
  name: string;
  avatarUrl?: string;
}

/** What RobotServer caches locally to validate PIN logins offline. */
export interface RosterEntryDto {
  assignmentId: string;
  userId: string;
  email: string;
  name: string;
  avatarUrl?: string;
  pinHash: string | null;
  pinSet: boolean;
  pinUpdatedCount: number;
}

export function toRosterEntryDto(
  assignment: IRobotAssignmentDocument,
  user: MinimalUser,
): RosterEntryDto {
  return {
    assignmentId: assignment._id.toString(),
    userId: user._id.toString(),
    email: user.email,
    name: user.name,
    avatarUrl: user.avatarUrl,
    pinHash: assignment.pinHash ?? null,
    pinSet: !!assignment.pinSetAt,
    pinUpdatedCount: assignment.pinUpdatedCount,
  };
}
