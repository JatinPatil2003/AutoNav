import path from "node:path";
import { RosUnavailableError } from "../../common/errors";
import { LaunchManager } from "../../ros/launchManager";
import { runOnce } from "../../ros/runOnce";
import { TopicGateway } from "../../ros/rosTopicGateway";
import {
  mapOccupancyGridToMapUpdate,
  mapPoseStampedToLocationUpdate,
  mapPoseToLocationUpdate,
  OccupancyGridMsg,
  PoseMsg,
  PoseStampedMsg,
} from "../telemetry/telemetry.mappers";
import { IGoalRepository } from "../goals/goal.repository.interface";
import { IMapRepository } from "../maps/map.repository.interface";
import { DEFAULT_SLAM_BACKEND, SLAM_LAUNCH_FILE, SlamBackend } from "./mapping.types";

const TEMP_MAP_NAME = "temp";
const MAPPING_LAUNCH_NAME = "mapping";
const MAP_TOPIC = "/map";
const AMCL_POSE_TOPIC = "/amcl_pose";
const SLAM_BRIDGE_POSE_TOPIC = "/pose_bridge/map_pose";

export class MappingService {
  /** Only meaningful while launchManager.status().name === "mapping" — see getActiveBackend(). */
  private currentBackend: SlamBackend | null = null;

  constructor(
    private readonly topicGateway: TopicGateway,
    private readonly goalRepository: IGoalRepository,
    private readonly mapRepository: IMapRepository,
    private readonly launchManager: LaunchManager,
    private readonly mapsDir: string,
  ) {}

  async start(backend: SlamBackend = DEFAULT_SLAM_BACKEND): Promise<void> {
    this.launchManager.start(MAPPING_LAUNCH_NAME, "ros2", ["launch", "autonav_navigation", SLAM_LAUNCH_FILE[backend]]);
    this.currentBackend = backend;
    await this.goalRepository.deleteByMap(TEMP_MAP_NAME);
  }

  stop(): void {
    this.launchManager.stop();
    this.currentBackend = null;
  }

  /** Null unless mapping is the launch actually running right now — guards against a stale backend value if something else (e.g. NavigationService.start()) stopped mapping via the shared LaunchManager without going through this.stop(). */
  getActiveBackend(): SlamBackend | null {
    return this.launchManager.status().name === MAPPING_LAUNCH_NAME ? this.currentBackend : null;
  }

  getCurrentMap() {
    const mapMsg = this.topicGateway.getLatest<OccupancyGridMsg>(MAP_TOPIC);
    return mapMsg ? mapOccupancyGridToMapUpdate(mapMsg) : null;
  }

  /**
   * AMCL isn't running during SLAM, so while mapping is active this prefers
   * the map->base_footprint TF republished as a topic by
   * autonav_utils's tf_pose_bridge node (launched alongside
   * both SLAM backends); otherwise falls back to AMCL.
   */
  getCurrentLocation() {
    if (this.launchManager.status().name === MAPPING_LAUNCH_NAME) {
      const bridgePose = this.topicGateway.getLatest<PoseStampedMsg>(SLAM_BRIDGE_POSE_TOPIC);
      if (bridgePose) {
        return mapPoseStampedToLocationUpdate(bridgePose);
      }
    }
    const poseMsg = this.topicGateway.getLatest<PoseMsg>(AMCL_POSE_TOPIC);
    return poseMsg ? mapPoseToLocationUpdate(poseMsg) : null;
  }

  async captureWaypoint(name: string): Promise<void> {
    const location = this.getCurrentLocation();
    if (!location) {
      throw new RosUnavailableError("No live pose available to capture — tf_pose_bridge hasn't published a map->base_footprint transform yet");
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
