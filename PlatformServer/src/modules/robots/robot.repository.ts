import { IRobotDocument, RobotModel, RobotStatus } from "./robot.model";
import { CreateRobotInput, IRobotRepository } from "./robot.repository.interface";

export class MongooseRobotRepository implements IRobotRepository {
  async findById(id: string): Promise<IRobotDocument | null> {
    return RobotModel.findById(id).exec();
  }

  async findByName(name: string): Promise<IRobotDocument | null> {
    return RobotModel.findOne({ name }).exec();
  }

  async listByStatus(status: RobotStatus): Promise<IRobotDocument[]> {
    return RobotModel.find({ status }).sort({ createdAt: -1 }).exec();
  }

  async listAll(): Promise<IRobotDocument[]> {
    return RobotModel.find().sort({ createdAt: -1 }).exec();
  }

  async create(input: CreateRobotInput): Promise<IRobotDocument> {
    const robot = new RobotModel({
      name: input.name,
      vpnHostname: input.vpnHostname,
      displayName: input.displayName,
      status: "unassigned",
    });
    return robot.save();
  }

  async assignToUser(robotId: string, userId: string): Promise<IRobotDocument | null> {
    return RobotModel.findByIdAndUpdate(
      robotId,
      { $set: { assignedTo: userId, status: "assigned" } },
      { new: true },
    ).exec();
  }
}
