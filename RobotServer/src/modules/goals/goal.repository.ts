import { GoalModel } from "./goal.model";
import { IGoalRepository } from "./goal.repository.interface";
import { Pose } from "../navigation/navigation.types";

export class MongooseGoalRepository implements IGoalRepository {
  async save(mapName: string, name: string, pose: Pose): Promise<void> {
    await GoalModel.create({ mapName, name, ...pose });
  }

  async upsert(mapName: string, name: string, pose: Pose): Promise<void> {
    await GoalModel.updateOne({ mapName, name }, { $set: { ...pose } }, { upsert: true }).exec();
  }

  async findOne(mapName: string, name: string): Promise<Pose | null> {
    const goal = await GoalModel.findOne({ mapName, name }).exec();
    if (!goal) {
      return null;
    }
    return { x: goal.x, y: goal.y, theta: goal.theta };
  }

  async listNames(mapName: string): Promise<string[]> {
    const goals = await GoalModel.find({ mapName }).select("name").exec();
    return goals.map((goal) => goal.name);
  }

  async reassignMap(fromMapName: string, toMapName: string): Promise<number> {
    const tempGoals = await GoalModel.find({ mapName: fromMapName }).exec();
    let processed = 0;

    for (const goal of tempGoals) {
      const existing = await GoalModel.findOne({ mapName: toMapName, name: goal.name }).exec();
      if (existing) {
        existing.x = goal.x;
        existing.y = goal.y;
        existing.theta = goal.theta;
        await existing.save();
      } else {
        goal.mapName = toMapName;
        await goal.save();
      }
      processed += 1;
    }

    return processed;
  }

  async deleteByMap(mapName: string): Promise<number> {
    const result = await GoalModel.deleteMany({ mapName }).exec();
    return result.deletedCount ?? 0;
  }
}
