import {
  IRobotAccessRequestDocument,
  RobotAccessRequestModel,
  RobotAccessRequestStatus,
} from "./robotAccessRequest.model";
import { IRobotAccessRequestRepository } from "./robotAccessRequest.repository.interface";

export class MongooseRobotAccessRequestRepository implements IRobotAccessRequestRepository {
  async create(userId: string): Promise<IRobotAccessRequestDocument> {
    const request = new RobotAccessRequestModel({ user: userId, status: "pending" });
    return request.save();
  }

  async findById(id: string): Promise<IRobotAccessRequestDocument | null> {
    return RobotAccessRequestModel.findById(id).exec();
  }

  async findLatestForUser(userId: string): Promise<IRobotAccessRequestDocument | null> {
    return RobotAccessRequestModel.findOne({ user: userId }).sort({ createdAt: -1 }).exec();
  }

  async findPendingForUser(userId: string): Promise<IRobotAccessRequestDocument | null> {
    return RobotAccessRequestModel.findOne({ user: userId, status: "pending" }).exec();
  }

  async listByStatus(status: RobotAccessRequestStatus): Promise<IRobotAccessRequestDocument[]> {
    return RobotAccessRequestModel.find({ status })
      .sort({ createdAt: 1 })
      .populate("user", "email name")
      .exec();
  }

  async markApproved(id: string, reviewedBy: string): Promise<IRobotAccessRequestDocument | null> {
    return RobotAccessRequestModel.findByIdAndUpdate(
      id,
      { $set: { status: "approved", reviewedBy, reviewedAt: new Date() } },
      { new: true },
    ).exec();
  }

  async markRejected(
    id: string,
    reviewedBy: string,
    reason?: string,
  ): Promise<IRobotAccessRequestDocument | null> {
    return RobotAccessRequestModel.findByIdAndUpdate(
      id,
      { $set: { status: "rejected", reviewedBy, reviewedAt: new Date(), rejectionReason: reason } },
      { new: true },
    ).exec();
  }
}
