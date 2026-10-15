import { IRobotAssignmentDocument, RobotAssignmentModel } from "./robotAssignment.model";
import { IRobotAssignmentRepository } from "./robotAssignment.repository.interface";

export class MongooseRobotAssignmentRepository implements IRobotAssignmentRepository {
  async findById(id: string): Promise<IRobotAssignmentDocument | null> {
    return RobotAssignmentModel.findById(id).exec();
  }

  async findByUserAndRobot(userId: string, robotId: string): Promise<IRobotAssignmentDocument | null> {
    return RobotAssignmentModel.findOne({ user: userId, robot: robotId }).exec();
  }

  async findActiveByUserAndRobot(
    userId: string,
    robotId: string,
  ): Promise<IRobotAssignmentDocument | null> {
    return RobotAssignmentModel.findOne({ user: userId, robot: robotId, status: "active" }).exec();
  }

  async listActiveForUser(userId: string): Promise<IRobotAssignmentDocument[]> {
    return RobotAssignmentModel.find({ user: userId, status: "active", platformAccess: true })
      .populate("robot")
      .sort({ grantedAt: -1 })
      .exec();
  }

  async listForRobot(robotId: string): Promise<IRobotAssignmentDocument[]> {
    return RobotAssignmentModel.find({ robot: robotId })
      .populate("user", "email name avatarUrl")
      .sort({ createdAt: -1 })
      .exec();
  }

  async listForUser(userId: string): Promise<IRobotAssignmentDocument[]> {
    return RobotAssignmentModel.find({ user: userId })
      .populate("robot")
      .sort({ createdAt: -1 })
      .exec();
  }

  async listActiveForRobotWithPin(robotId: string): Promise<IRobotAssignmentDocument[]> {
    return RobotAssignmentModel.find({ robot: robotId, status: "active", hmiAccess: true })
      .select("+pinHash")
      .populate("user", "email name avatarUrl")
      .exec();
  }

  async upsertActive(
    userId: string,
    robotId: string,
    grantedBy: string,
    access?: { hmiAccess?: boolean; platformAccess?: boolean },
  ): Promise<IRobotAssignmentDocument> {
    const existing = await RobotAssignmentModel.findOne({ user: userId, robot: robotId }).exec();
    if (existing) {
      existing.status = "active";
      existing.grantedBy = grantedBy as unknown as IRobotAssignmentDocument["grantedBy"];
      existing.grantedAt = new Date();
      existing.revokedBy = undefined;
      existing.revokedAt = undefined;
      existing.revokedReason = undefined;
      if (access?.hmiAccess !== undefined) existing.hmiAccess = access.hmiAccess;
      if (access?.platformAccess !== undefined) existing.platformAccess = access.platformAccess;
      return existing.save();
    }

    const assignment = new RobotAssignmentModel({
      user: userId,
      robot: robotId,
      status: "active",
      grantedBy,
      grantedAt: new Date(),
      hmiAccess: access?.hmiAccess ?? true,
      platformAccess: access?.platformAccess ?? true,
    });
    return assignment.save();
  }

  async revoke(id: string, revokedBy: string, reason?: string): Promise<IRobotAssignmentDocument | null> {
    return RobotAssignmentModel.findByIdAndUpdate(
      id,
      { $set: { status: "revoked", revokedBy, revokedAt: new Date(), revokedReason: reason } },
      { new: true },
    ).exec();
  }

  async setPin(id: string, pinHash: string): Promise<IRobotAssignmentDocument | null> {
    return RobotAssignmentModel.findByIdAndUpdate(
      id,
      { $set: { pinHash, pinSetAt: new Date() }, $inc: { pinUpdatedCount: 1 } },
      { new: true },
    ).exec();
  }

  async updateAccess(
    id: string,
    access: { hmiAccess?: boolean; platformAccess?: boolean },
  ): Promise<IRobotAssignmentDocument | null> {
    return RobotAssignmentModel.findByIdAndUpdate(id, { $set: access }, { new: true }).exec();
  }

  async deleteForRobot(robotId: string): Promise<void> {
    await RobotAssignmentModel.deleteMany({ robot: robotId }).exec();
  }

  async deleteForUser(userId: string): Promise<void> {
    await RobotAssignmentModel.deleteMany({ user: userId }).exec();
  }
}
