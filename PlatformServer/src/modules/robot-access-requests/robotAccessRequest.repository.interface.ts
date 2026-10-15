import { IRobotAccessRequestDocument, RobotAccessRequestStatus } from "./robotAccessRequest.model";

export interface IRobotAccessRequestRepository {
  create(userId: string, robotId: string): Promise<IRobotAccessRequestDocument>;
  findById(id: string): Promise<IRobotAccessRequestDocument | null>;
  listForUser(userId: string): Promise<IRobotAccessRequestDocument[]>;
  findPendingForUserAndRobot(userId: string, robotId: string): Promise<IRobotAccessRequestDocument | null>;
  listByStatus(status: RobotAccessRequestStatus): Promise<IRobotAccessRequestDocument[]>;
  markApproved(id: string, reviewedBy: string): Promise<IRobotAccessRequestDocument | null>;
  markRejected(
    id: string,
    reviewedBy: string,
    reason?: string,
  ): Promise<IRobotAccessRequestDocument | null>;
  /** Hard-deletes every request (any status) for a robot being deleted. */
  deleteForRobot(robotId: string): Promise<void>;
  /** Hard-deletes every request (any status) for a user being deleted. */
  deleteForUser(userId: string): Promise<void>;
}
