import { IGoalRepository } from "../../../src/modules/goals/goal.repository.interface";
import { Pose } from "../../../src/modules/navigation/navigation.types";

export class FakeGoalRepository implements IGoalRepository {
  private readonly goals = new Map<string, Pose>();

  private key(mapName: string, name: string): string {
    return `${mapName}::${name}`;
  }

  async save(mapName: string, name: string, pose: Pose): Promise<void> {
    this.goals.set(this.key(mapName, name), pose);
  }

  async upsert(mapName: string, name: string, pose: Pose): Promise<void> {
    this.goals.set(this.key(mapName, name), pose);
  }

  async findOne(mapName: string, name: string): Promise<Pose | null> {
    return this.goals.get(this.key(mapName, name)) ?? null;
  }

  async listNames(mapName: string): Promise<string[]> {
    const prefix = `${mapName}::`;
    return [...this.goals.keys()].filter((key) => key.startsWith(prefix)).map((key) => key.slice(prefix.length));
  }

  async reassignMap(fromMapName: string, toMapName: string): Promise<number> {
    const names = await this.listNames(fromMapName);
    for (const name of names) {
      const pose = await this.findOne(fromMapName, name);
      if (pose) {
        this.goals.set(this.key(toMapName, name), pose);
      }
    }
    return names.length;
  }

  async deleteByMap(mapName: string): Promise<number> {
    const names = await this.listNames(mapName);
    for (const name of names) {
      this.goals.delete(this.key(mapName, name));
    }
    return names.length;
  }
}
