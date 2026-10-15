import path from "node:path";
import { RosUnavailableError } from "../../common/errors";
import { LaunchManager } from "../../ros/launchManager";
import { runOnce } from "../../ros/runOnce";
import { TopicGateway } from "../../ros/rosTopicGateway";
import { mapOccupancyGridToMapUpdate, mapPoseToLocationUpdate, OccupancyGridMsg, PoseMsg } from "../telemetry/telemetry.mappers";
import { IGoalRepository } from "../goals/goal.repository.interface";
import { IMapRepository } from "../maps/map.repository.interface";

const TEMP_MAP_NAME = "temp";
const MAP_TOPIC = "/map";
const AMCL_POSE_TOPIC = "/amcl_pose";

export class MappingService {
  constructor(
    private readonly topicGateway: TopicGateway,
    private readonly goalRepository: IGoalRepository,
    private readonly mapRepository: IMapRepository,
    private readonly launchManager: LaunchManager,
    private readonly mapsDir: string,
  ) {}

  async start(): Promise<void> {
    this.launchManager.start("mapping", "ros2", ["launch", "autonav_navigation", "slam.launch.py"]);
    await this.goalRepository.deleteByMap(TEMP_MAP_NAME);
  }

  stop(): void {
    this.launchManager.stop();
  }

  getCurrentMap() {
    const mapMsg = this.topicGateway.getLatest<OccupancyGridMsg>(MAP_TOPIC);
    return mapMsg ? mapOccupancyGridToMapUpdate(mapMsg) : null;
  }

  /**
   * AMCL isn't running during SLAM, so this relies on the TF2 gap noted in
   * telemetry.broadcaster.ts and will be null until that's resolved.
   */
  getCurrentLocation() {
    const poseMsg = this.topicGateway.getLatest<PoseMsg>(AMCL_POSE_TOPIC);
    return poseMsg ? mapPoseToLocationUpdate(poseMsg) : null;
  }

  async captureWaypoint(name: string): Promise<void> {
    const location = this.getCurrentLocation();
    if (!location) {
      throw new RosUnavailableError(
        "No live pose available to capture — this needs the TF2 lookup noted in telemetry.broadcaster.ts, not yet implemented",
      );
    }
    await this.goalRepository.upsert(TEMP_MAP_NAME, name, location);
  }

  async saveMap(mapName: string): Promise<void> {
    const outputPrefix = path.join(this.mapsDir, mapName);
    const exitCode = await runOnce("ros2", [
      "run",
      "nav2_map_server",
      "map_saver_cli",
      "--free",
      "0.15",
      "--fmt",
      "png",
      "-f",
      outputPrefix,
    ]);

    if (exitCode !== 0) {
      throw new RosUnavailableError(`map_saver_cli exited with code ${exitCode}`);
    }

    await this.mapRepository.save(mapName, this.mapsDir);
  }

  async commitWaypoints(mapName: string): Promise<number> {
    const processed = await this.goalRepository.reassignMap(TEMP_MAP_NAME, mapName);
    await this.goalRepository.deleteByMap(TEMP_MAP_NAME);
    return processed;
  }
}
