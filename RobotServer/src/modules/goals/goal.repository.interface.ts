import { Pose } from "../navigation/navigation.types";

export interface IGoalRepository {
  save(mapName: string, name: string, pose: Pose): Promise<void>;
  upsert(mapName: string, name: string, pose: Pose): Promise<void>;
  findOne(mapName: string, name: string): Promise<Pose | null>;
  listNames(mapName: string): Promise<string[]>;
  /** Moves every goal from `fromMapName` to `toMapName`, merging onto any existing name collisions. Returns the number processed. */
  reassignMap(fromMapName: string, toMapName: string): Promise<number>;
  /** Returns the number of goals deleted. */
  deleteByMap(mapName: string): Promise<number>;
}
