import {
  IRobotAccessRequestDocument,
  RobotAccessRequestModel,
  RobotAccessRequestStatus,
} from "./robotAccessRequest.model";
import { IRobotAccessRequestRepository } from "./robotAccessRequest.repository.interface";

export class MongooseRobotAccessRequestRepository implements IRobotAccessRequestRepository {
  async create(userId: string, robotId: string): Promise<IRobotAccessRequestDocument> {
    const request = new RobotAccessRequestModel({ user: userId, robot: robotId, status: "pending" });
    return request.save();
  }

  async findById(id: string): Promise<IRobotAccessRequestDocument | null> {
    return RobotAccessRequestModel.findById(id).populate("robot").exec();
  }

  async listForUser(userId: string): Promise<IRobotAccessRequestDocument[]> {
    return RobotAccessRequestModel.find({ user: userId })
      .populate("robot")
      .sort({ createdAt: -1 })
      .exec();
  }

  async findPendingForUserAndRobot(
    userId: string,
    robotId: string,
  ): Promise<IRobotAccessRequestDocument | null> {
    return RobotAccessRequestModel.findOne({ user: userId, robot: robotId, status: "pending" }).exec();
  }

  async listByStatus(status: RobotAccessRequestStatus): Promise<IRobotAccessRequestDocument[]> {
    return RobotAccessRequestModel.find({ status })
      .sort({ createdAt: 1 })
      .populate("user", "email name")
      .populate("robot")
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

  async deleteForRobot(robotId: string): Promise<void> {
    await RobotAccessRequestModel.deleteMany({ robot: robotId }).exec();
  }

  async deleteForUser(userId: string): Promise<void> {
    await RobotAccessRequestModel.deleteMany({ user: userId }).exec();
  }
}
