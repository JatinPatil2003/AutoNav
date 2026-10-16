import { IRobotDocument, RobotModel, RobotStatus } from "./robot.model";
import { CreateRobotInput, IRobotRepository } from "./robot.repository.interface";

export class MongooseRobotRepository implements IRobotRepository {
  async findById(id: string): Promise<IRobotDocument | null> {
    return RobotModel.findById(id).exec();
  }

  async findByName(name: string): Promise<IRobotDocument | null> {
    return RobotModel.findOne({ name }).exec();
  }

  async findByRobotId(robotId: string): Promise<IRobotDocument | null> {
    return RobotModel.findOne({ robotId: robotId.toLowerCase() }).exec();
  }

  async findByRobotIdWithLinkSecret(robotId: string): Promise<IRobotDocument | null> {
    return RobotModel.findOne({ robotId: robotId.toLowerCase() })
      .select("+linkSecretHash")
      .exec();
  }

  async listByStatus(status: RobotStatus): Promise<IRobotDocument[]> {
    return RobotModel.find({ status }).sort({ createdAt: -1 }).exec();
  }

  async listAll(): Promise<IRobotDocument[]> {
    return RobotModel.find().sort({ createdAt: -1 }).exec();
  }

  async create(input: CreateRobotInput): Promise<IRobotDocument> {
    const robot = new RobotModel({
      robotId: input.robotId.toLowerCase(),
      name: input.name,
      vpnHostname: input.vpnHostname,
      displayName: input.displayName,
      status: "provisioned",
    });
    return robot.save();
  }

  async markSeen(robotId: string): Promise<void> {
    await RobotModel.updateOne(
      { robotId: robotId.toLowerCase() },
      { $set: { lastSeenAt: new Date(), status: "active" } },
    ).exec();
  }

  async touchLastSeen(robotId: string): Promise<void> {
    await RobotModel.updateOne(
      { robotId: robotId.toLowerCase() },
      { $set: { lastSeenAt: new Date() } },
    ).exec();
  }

  async deleteById(id: string): Promise<void> {
    await RobotModel.deleteOne({ _id: id }).exec();
  }

  async setLinkSecretHash(id: string, linkSecretHash: string): Promise<void> {
    await RobotModel.updateOne({ _id: id }, { $set: { linkSecretHash } }).exec();
  }
}
