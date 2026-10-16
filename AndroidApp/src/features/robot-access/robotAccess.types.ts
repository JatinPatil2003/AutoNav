import { RobotSummary } from "../auth/auth.types";

export type RobotAccessRequestStatus = "pending" | "approved" | "rejected";

export interface RobotAccessRequest {
  id: string;
  status: RobotAccessRequestStatus;
  requestedBy?: { id: string; email: string; name: string };
  robot?: RobotSummary;
  reviewedAt?: string;
  rejectionReason?: string;
  createdAt: string;
}

/** Minimal, non-admin-safe robot listing for the request-access picker. */
export interface RobotOption {
  id: string;
  robotId: string;
  name: string;
  displayName?: string;
}
