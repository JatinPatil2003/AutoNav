import { Types } from "mongoose";
import { IRobotDocument, RobotStatus } from "../../../src/modules/robots/robot.model";
import { CreateRobotInput, IRobotRepository } from "../../../src/modules/robots/robot.repository.interface";

export class FakeRobotRepository implements IRobotRepository {
  private robots = new Map<string, IRobotDocument>();

  async findById(id: string): Promise<IRobotDocument | null> {
    return this.robots.get(id) ?? null;
  }

  async findByName(name: string): Promise<IRobotDocument | null> {
    return [...this.robots.values()].find((r) => r.name === name) ?? null;
  }

  async findByRobotId(robotId: string): Promise<IRobotDocument | null> {
    return [...this.robots.values()].find((r) => r.robotId === robotId.toLowerCase()) ?? null;
  }

  async listByStatus(status: RobotStatus): Promise<IRobotDocument[]> {
    return [...this.robots.values()].filter((r) => r.status === status);
  }

  async listAll(): Promise<IRobotDocument[]> {
    return [...this.robots.values()];
  }

  async create(input: CreateRobotInput): Promise<IRobotDocument> {
    const id = new Types.ObjectId();
    const robot = {
      _id: id,
      robotId: input.robotId.toLowerCase(),
      name: input.name,
      vpnHostname: input.vpnHostname,
      displayName: input.displayName,
      status: "provisioned" as RobotStatus,
      createdAt: new Date(),
      updatedAt: new Date(),
    } as unknown as IRobotDocument;

    this.robots.set(id.toString(), robot);
    return robot;
  }

  async markSeen(robotId: string): Promise<void> {
    const robot = [...this.robots.values()].find((r) => r.robotId === robotId.toLowerCase());
    if (robot) {
      robot.lastSeenAt = new Date();
      robot.status = "active";
    }
  }

  async deleteById(id: string): Promise<void> {
    this.robots.delete(id);
  }
}
