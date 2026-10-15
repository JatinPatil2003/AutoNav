import { Types } from "mongoose";
import { IRobotAssignmentDocument } from "../../../src/modules/robot-assignments/robotAssignment.model";
import { IRobotAssignmentRepository } from "../../../src/modules/robot-assignments/robotAssignment.repository.interface";
import { FakeRobotRepository } from "./FakeRobotRepository";
import { FakeUserRepository } from "./FakeUserRepository";

/**
 * Mirrors Mongoose's .populate("robot")/.populate("user") behavior so the
 * service layer (which checks "is this field populated?" the same way it
 * would against a real document) sees the same shape in tests as in prod.
 */
export class FakeRobotAssignmentRepository implements IRobotAssignmentRepository {
  private assignments = new Map<string, IRobotAssignmentDocument>();

  constructor(
    private readonly robotRepository?: FakeRobotRepository,
    private readonly userRepository?: FakeUserRepository,
  ) {}

  private async populateRobot(assignment: IRobotAssignmentDocument): Promise<IRobotAssignmentDocument> {
    if (!this.robotRepository) return assignment;
    const robot = await this.robotRepository.findById(assignment.robot.toString());
    if (!robot) return assignment;
    return { ...assignment, robot } as unknown as IRobotAssignmentDocument;
  }

  private async populateUser(assignment: IRobotAssignmentDocument): Promise<IRobotAssignmentDocument> {
    if (!this.userRepository) return assignment;
    const user = await this.userRepository.findById(assignment.user.toString());
    if (!user) return assignment;
    return { ...assignment, user } as unknown as IRobotAssignmentDocument;
  }

  async findById(id: string): Promise<IRobotAssignmentDocument | null> {
    return this.assignments.get(id) ?? null;
  }

  async findByUserAndRobot(userId: string, robotId: string): Promise<IRobotAssignmentDocument | null> {
    return (
      [...this.assignments.values()].find(
        (a) => a.user.toString() === userId && a.robot.toString() === robotId,
      ) ?? null
    );
  }

  async findActiveByUserAndRobot(userId: string, robotId: string): Promise<IRobotAssignmentDocument | null> {
    return (
      [...this.assignments.values()].find(
        (a) => a.user.toString() === userId && a.robot.toString() === robotId && a.status === "active",
      ) ?? null
    );
  }

  async listActiveForUser(userId: string): Promise<IRobotAssignmentDocument[]> {
    const matches = [...this.assignments.values()].filter(
      (a) => a.user.toString() === userId && a.status === "active" && a.platformAccess,
    );
    return Promise.all(matches.map((a) => this.populateRobot(a)));
  }

  async listForRobot(robotId: string): Promise<IRobotAssignmentDocument[]> {
    const matches = [...this.assignments.values()].filter((a) => a.robot.toString() === robotId);
    return Promise.all(matches.map((a) => this.populateUser(a)));
  }

  async listForUser(userId: string): Promise<IRobotAssignmentDocument[]> {
    const matches = [...this.assignments.values()].filter((a) => a.user.toString() === userId);
    return Promise.all(matches.map((a) => this.populateRobot(a)));
  }

  async listActiveForRobotWithPin(robotId: string): Promise<IRobotAssignmentDocument[]> {
    const matches = [...this.assignments.values()].filter(
      (a) => a.robot.toString() === robotId && a.status === "active" && a.hmiAccess,
    );
    return Promise.all(matches.map((a) => this.populateUser(a)));
  }

  async upsertActive(
    userId: string,
    robotId: string,
    grantedBy: string,
    access?: { hmiAccess?: boolean; platformAccess?: boolean },
  ): Promise<IRobotAssignmentDocument> {
    const existing = await this.findByUserAndRobot(userId, robotId);
    if (existing) {
      existing.status = "active";
      existing.grantedBy = new Types.ObjectId(grantedBy);
      existing.grantedAt = new Date();
      existing.revokedBy = undefined;
      existing.revokedAt = undefined;
      existing.revokedReason = undefined;
      if (access?.hmiAccess !== undefined) existing.hmiAccess = access.hmiAccess;
      if (access?.platformAccess !== undefined) existing.platformAccess = access.platformAccess;
      return existing;
    }

    const id = new Types.ObjectId();
    const assignment = {
      _id: id,
      user: new Types.ObjectId(userId),
      robot: new Types.ObjectId(robotId),
      status: "active",
      hmiAccess: access?.hmiAccess ?? true,
      platformAccess: access?.platformAccess ?? true,
      pinUpdatedCount: 0,
      grantedBy: new Types.ObjectId(grantedBy),
      grantedAt: new Date(),
      createdAt: new Date(),
      updatedAt: new Date(),
    } as unknown as IRobotAssignmentDocument;

    this.assignments.set(id.toString(), assignment);
    return assignment;
  }

  async revoke(id: string, revokedBy: string, reason?: string): Promise<IRobotAssignmentDocument | null> {
    const assignment = this.assignments.get(id);
    if (!assignment) return null;
    assignment.status = "revoked";
    assignment.revokedBy = new Types.ObjectId(revokedBy);
    assignment.revokedAt = new Date();
    assignment.revokedReason = reason;
    return assignment;
  }

  async setPin(id: string, pinHash: string): Promise<IRobotAssignmentDocument | null> {
    const assignment = this.assignments.get(id);
    if (!assignment) return null;
    assignment.pinHash = pinHash;
    assignment.pinSetAt = new Date();
    assignment.pinUpdatedCount += 1;
    return assignment;
  }

  async updateAccess(
    id: string,
    access: { hmiAccess?: boolean; platformAccess?: boolean },
  ): Promise<IRobotAssignmentDocument | null> {
    const assignment = this.assignments.get(id);
    if (!assignment) return null;
    if (access.hmiAccess !== undefined) assignment.hmiAccess = access.hmiAccess;
    if (access.platformAccess !== undefined) assignment.platformAccess = access.platformAccess;
    return assignment;
  }

  async deleteForRobot(robotId: string): Promise<void> {
    for (const [id, assignment] of this.assignments) {
      if (assignment.robot.toString() === robotId) this.assignments.delete(id);
    }
  }

  async deleteForUser(userId: string): Promise<void> {
    for (const [id, assignment] of this.assignments) {
      if (assignment.user.toString() === userId) this.assignments.delete(id);
    }
  }
}
