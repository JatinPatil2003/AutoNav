import { IRobotAccessRequestDocument, RobotAccessRequestStatus } from "./robotAccessRequest.model";

export interface IRobotAccessRequestRepository {
  create(userId: string): Promise<IRobotAccessRequestDocument>;
  findById(id: string): Promise<IRobotAccessRequestDocument | null>;
  findLatestForUser(userId: string): Promise<IRobotAccessRequestDocument | null>;
  findPendingForUser(userId: string): Promise<IRobotAccessRequestDocument | null>;
  listByStatus(status: RobotAccessRequestStatus): Promise<IRobotAccessRequestDocument[]>;
  markApproved(id: string, reviewedBy: string): Promise<IRobotAccessRequestDocument | null>;
  markRejected(
    id: string,
    reviewedBy: string,
    reason?: string,
  ): Promise<IRobotAccessRequestDocument | null>;
}
