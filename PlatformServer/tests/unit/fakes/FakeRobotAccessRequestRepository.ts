import { Types } from "mongoose";
import {
  IRobotAccessRequestDocument,
  RobotAccessRequestStatus,
} from "../../../src/modules/robot-access-requests/robotAccessRequest.model";
import { IRobotAccessRequestRepository } from "../../../src/modules/robot-access-requests/robotAccessRequest.repository.interface";

export class FakeRobotAccessRequestRepository implements IRobotAccessRequestRepository {
  private requests = new Map<string, IRobotAccessRequestDocument>();

  async create(userId: string, robotId: string): Promise<IRobotAccessRequestDocument> {
    const hasPending = [...this.requests.values()].some(
      (r) => r.user.toString() === userId && r.robot.toString() === robotId && r.status === "pending",
    );

    if (hasPending) {
      const err = new Error("duplicate pending request") as Error & { code: number };
      err.code = 11000;
      throw err;
    }

    const id = new Types.ObjectId();
    const request = {
      _id: id,
      user: new Types.ObjectId(userId),
      robot: new Types.ObjectId(robotId),
      status: "pending" as RobotAccessRequestStatus,
      createdAt: new Date(),
      updatedAt: new Date(),
    } as unknown as IRobotAccessRequestDocument;

    this.requests.set(id.toString(), request);
    return request;
  }

  async findById(id: string): Promise<IRobotAccessRequestDocument | null> {
    return this.requests.get(id) ?? null;
  }

  async listForUser(userId: string): Promise<IRobotAccessRequestDocument[]> {
    return [...this.requests.values()]
      .filter((r) => r.user.toString() === userId)
      .sort((a, b) => b.createdAt.getTime() - a.createdAt.getTime());
  }

  async findPendingForUserAndRobot(
    userId: string,
    robotId: string,
  ): Promise<IRobotAccessRequestDocument | null> {
    return (
      [...this.requests.values()].find(
        (r) => r.user.toString() === userId && r.robot.toString() === robotId && r.status === "pending",
      ) ?? null
    );
  }

  async listByStatus(status: RobotAccessRequestStatus): Promise<IRobotAccessRequestDocument[]> {
    return [...this.requests.values()].filter((r) => r.status === status);
  }

  async markApproved(id: string, reviewedBy: string): Promise<IRobotAccessRequestDocument | null> {
    const request = this.requests.get(id);
    if (!request) return null;
    request.status = "approved";
    request.reviewedBy = new Types.ObjectId(reviewedBy);
    request.reviewedAt = new Date();
    return request;
  }

  async markRejected(
    id: string,
    reviewedBy: string,
    reason?: string,
  ): Promise<IRobotAccessRequestDocument | null> {
    const request = this.requests.get(id);
    if (!request) return null;
    request.status = "rejected";
    request.reviewedBy = new Types.ObjectId(reviewedBy);
    request.reviewedAt = new Date();
    request.rejectionReason = reason;
    return request;
  }

  async deleteForRobot(robotId: string): Promise<void> {
    for (const [id, request] of this.requests) {
      if (request.robot.toString() === robotId) this.requests.delete(id);
    }
  }

  async deleteForUser(userId: string): Promise<void> {
    for (const [id, request] of this.requests) {
      if (request.user.toString() === userId) this.requests.delete(id);
    }
  }
}
