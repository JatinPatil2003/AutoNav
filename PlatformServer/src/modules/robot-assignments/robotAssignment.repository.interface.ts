import { IRobotAssignmentDocument } from "./robotAssignment.model";

export interface IRobotAssignmentRepository {
  findById(id: string): Promise<IRobotAssignmentDocument | null>;
  /** Any status — used to decide whether to create vs. reactivate. */
  findByUserAndRobot(userId: string, robotId: string): Promise<IRobotAssignmentDocument | null>;
  findActiveByUserAndRobot(userId: string, robotId: string): Promise<IRobotAssignmentDocument | null>;
  /** Populated with `robot`, filtered to platformAccess — the current user's "my robots" view. */
  listActiveForUser(userId: string): Promise<IRobotAssignmentDocument[]>;
  /** All statuses, populated with `user`, for admin visibility. */
  listForRobot(robotId: string): Promise<IRobotAssignmentDocument[]>;
  listForUser(userId: string): Promise<IRobotAssignmentDocument[]>;
  /** Active + hmiAccess only, with pinHash selected — the roster RobotServer fetches. */
  listActiveForRobotWithPin(robotId: string): Promise<IRobotAssignmentDocument[]>;
  /** Creates a new active assignment, or reactivates an existing revoked one. */
  upsertActive(
    userId: string,
    robotId: string,
    grantedBy: string,
    access?: { hmiAccess?: boolean; platformAccess?: boolean },
  ): Promise<IRobotAssignmentDocument>;
  revoke(id: string, revokedBy: string, reason?: string): Promise<IRobotAssignmentDocument | null>;
  setPin(id: string, pinHash: string): Promise<IRobotAssignmentDocument | null>;
  /** Partial update of the per-assignment access flags, without touching status/pin. */
  updateAccess(
    id: string,
    access: { hmiAccess?: boolean; platformAccess?: boolean },
  ): Promise<IRobotAssignmentDocument | null>;
  /** Hard-deletes every assignment (any status) for a robot being deleted. */
  deleteForRobot(robotId: string): Promise<void>;
  /** Hard-deletes every assignment (any status) for a user being deleted. */
  deleteForUser(userId: string): Promise<void>;
}
