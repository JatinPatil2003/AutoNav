export type RobotAccessRequestStatus = "pending" | "approved" | "rejected";

export interface RobotAccessRequest {
  id: string;
  status: RobotAccessRequestStatus;
  requestedBy?: { id: string; email: string; name: string };
  reviewedAt?: string;
  rejectionReason?: string;
  createdAt: string;
}
