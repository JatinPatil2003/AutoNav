import path from "node:path";
import * as rclnodejs from "rclnodejs";
import { NotFoundError, RosUnavailableError } from "../../common/errors";
import { logger } from "../../common/utils/logger.util";
import { LaunchManager } from "../../ros/launchManager";
import { quaternionFromYaw } from "../../ros/quaternion";
import { ActionGateway } from "../../ros/rosActionGateway";
import { ServiceGateway } from "../../ros/rosServiceGateway";
import { TopicGateway } from "../../ros/rosTopicGateway";
import { TeleopService } from "../teleop/teleop.service";
import { isPoseConfident, mapOccupancyGridToMapUpdate, mapPoseToLocationUpdate, OccupancyGridMsg, PoseMsg } from "../telemetry/telemetry.mappers";
import { IGoalRepository } from "../goals/goal.repository.interface";
import { IMapRepository } from "../maps/map.repository.interface";
import { NavStatus, Pose } from "./navigation.types";

export interface CategorizedGoals {
  localization: string[];
  charging: string[];
  standby: string[];
  goals: string[];
}

const ACTION_WAIT_TIMEOUT_MS = 1000;
const SERVICE_WAIT_TIMEOUT_MS = 1000;
const NAVIGATE_TO_POSE_ACTION = "navigate_to_pose";
const SET_INITIAL_POSE_SERVICE = "/set_initial_pose";
const LOAD_MAP_SERVICE = "/map_server/load_map";
const MAP_TOPIC = "/map";
const AMCL_POSE_TOPIC = "/amcl_pose";

// header.stamp is left at zero rather than read from the node's clock —
// Nav2/AMCL treat an unspecified stamp as "now" for these request types,
// and neither hard-rejects it.
const UNSPECIFIED_STAMP = { sec: 0, nanosec: 0 };

interface NavigateFeedback {
  distance_remaining: number;
}

interface LoadMapResponse {
  result: number; // LoadMap.Response.RESULT_SUCCESS == 0
}

export class NavigationService {
  private goalHandle: rclnodejs.ClientGoalHandle<any> | undefined;
  private status: NavStatus = NavStatus.IDLE;
  private feedback = 0;
  private activeGoalName: string | null = null;

  constructor(
    private readonly actionGateway: ActionGateway,
    private readonly serviceGateway: ServiceGateway,
    private readonly topicGateway: TopicGateway,
    private readonly teleopService: TeleopService,
    private readonly goalRepository: IGoalRepository,
    private readonly mapRepository: IMapRepository,
    private readonly launchManager: LaunchManager,
    private readonly mapsDir: string,
  ) {}

  async start(mapName: string): Promise<void> {
    const found = await this.mapRepository.load(mapName, this.mapsDir);
    if (!found) {
      throw new NotFoundError(`Map "${mapName}" was not found`);
    }

    this.launchManager.start("navigation", "ros2", [
      "launch",
      "autonav_navigation",
      "navigation.launch.py",
      `map_name:=${mapName}.yaml`,
    ]);
  }

  stop(): void {
    this.launchManager.stop();
  }

  async listMaps(): Promise<string[]> {
    return this.mapRepository.list();
  }

  async selectMap(mapName: string): Promise<void> {
    const found = await this.mapRepository.load(mapName, this.mapsDir);
    if (!found) {
      throw new NotFoundError(`Map "${mapName}" was not found`);
    }

    const mapYamlPath = path.join(this.mapsDir, `${mapName}.yaml`);
    const response = await this.serviceGateway.call<{ map_url: string }, LoadMapResponse>(
      "nav2_msgs/srv/LoadMap",
      LOAD_MAP_SERVICE,
      { map_url: mapYamlPath },
      SERVICE_WAIT_TIMEOUT_MS,
    );
    if (response.result !== 0) {
      throw new RosUnavailableError("map_server did not accept the map reload");
    }
  }

  async listGoals(mapName: string): Promise<CategorizedGoals> {
    const names = await this.goalRepository.listNames(mapName);
    const categorized: CategorizedGoals = { localization: [], charging: [], standby: [], goals: [] };

    for (const name of names) {
      const lower = name.toLowerCase();
      if (lower.includes("localization")) categorized.localization.push(name);
      else if (lower.includes("charger")) categorized.charging.push(name);
      else if (lower.includes("standby")) categorized.standby.push(name);
      else categorized.goals.push(name);
    }

    return categorized;
  }

  async saveGoal(mapName: string, name: string, pose: Pose): Promise<void> {
    await this.goalRepository.save(mapName, name, pose);
  }

  async getGoal(mapName: string, name: string): Promise<Pose> {
    const pose = await this.goalRepository.findOne(mapName, name);
    if (!pose) {
      throw new NotFoundError(`Goal "${name}" was not found on map "${mapName}"`);
    }
    return pose;
  }

  async localize(mapName: string, name: string): Promise<{ success: boolean; message: string }> {
    const pose = await this.getGoal(mapName, name);
    await this.setInitialPose(pose);
    await this.teleopService.rotateInPlace(2);

    const poseMsg = this.topicGateway.getLatest<PoseMsg>(AMCL_POSE_TOPIC);
    if (isPoseConfident(poseMsg)) {
      return { success: true, message: `Localization to ${name} successful` };
    }
    return { success: false, message: `Localization failed at ${name}, please retry` };
  }

  async startGoal(mapName: string, name: string): Promise<void> {
    const pose = await this.getGoal(mapName, name);
    this.activeGoalName = name;
    await this.sendGoal(pose);
  }

  async cancelGoal(): Promise<void> {
    if (!this.goalHandle) {
      return;
    }
    await this.actionGateway.cancelGoal(this.goalHandle);
    this.status = NavStatus.CANCELED;
    this.feedback = 0;
    this.activeGoalName = null;
    this.goalHandle = undefined;
  }

  getFeedback(): number {
    return this.feedback;
  }

  getStatus(): NavStatus {
    return this.status;
  }

  /** Null once the goal reaches a terminal state — mirrors getStatus()/getFeedback()'s lifecycle. */
  getActiveGoalName(): string | null {
    return this.status === NavStatus.ACTIVE ? this.activeGoalName : null;
  }

  async setInitialPose(pose: Pose): Promise<void> {
    const quat = quaternionFromYaw(pose.theta);
    await this.serviceGateway.call(
      "nav2_msgs/srv/SetInitialPose",
      SET_INITIAL_POSE_SERVICE,
      {
        pose: {
          header: { frame_id: "map", stamp: UNSPECIFIED_STAMP },
          pose: {
            pose: {
              position: { x: pose.x, y: pose.y, z: 0 },
              orientation: { x: 0, y: 0, z: quat.z, w: quat.w },
            },
            covariance: new Array(36).fill(0),
          },
        },
      },
      SERVICE_WAIT_TIMEOUT_MS,
    );
  }

  getCurrentMap() {
    const mapMsg = this.topicGateway.getLatest<OccupancyGridMsg>(MAP_TOPIC);
    return mapMsg ? mapOccupancyGridToMapUpdate(mapMsg) : null;
  }

  getCurrentLocation() {
    const poseMsg = this.topicGateway.getLatest<PoseMsg>(AMCL_POSE_TOPIC);
    return poseMsg ? mapPoseToLocationUpdate(poseMsg) : null;
  }

  private async sendGoal(pose: Pose): Promise<void> {
    if (this.goalHandle) {
      await this.cancelGoal();
    }

    const quat = quaternionFromYaw(pose.theta);
    const goalMsg = {
      pose: {
        header: { frame_id: "map", stamp: UNSPECIFIED_STAMP },
        pose: {
          position: { x: pose.x, y: pose.y, z: 0 },
          orientation: { x: 0, y: 0, z: quat.z, w: quat.w },
        },
      },
      behavior_tree: "",
    };

    this.status = NavStatus.ACTIVE;
    this.feedback = 0;

    const { handle, result } = await this.actionGateway.sendGoal<typeof goalMsg, NavigateFeedback, unknown>(
      "nav2_msgs/action/NavigateToPose",
      NAVIGATE_TO_POSE_ACTION,
      goalMsg,
      (feedbackMsg) => {
        this.feedback = feedbackMsg.distance_remaining;
      },
      ACTION_WAIT_TIMEOUT_MS,
    );
    this.goalHandle = handle;

    result
      .then(() => {
        this.status = mapActionStatus(handle.status);
        this.feedback = 0;
        if (this.goalHandle === handle) {
          this.goalHandle = undefined;
        }
      })
      .catch((err: unknown) => {
        logger.error({ err }, "Navigation goal failed");
        this.status = NavStatus.FAILED;
        if (this.goalHandle === handle) {
          this.goalHandle = undefined;
        }
      });
  }
}

// GoalStatus constants from action_msgs/msg/GoalStatus
function mapActionStatus(status: number): NavStatus {
  switch (status) {
    case 4: // STATUS_SUCCEEDED
      return NavStatus.SUCCEEDED;
    case 6: // STATUS_ABORTED
      return NavStatus.FAILED;
    case 5: // STATUS_CANCELED
      return NavStatus.CANCELED;
    default:
      return NavStatus.IDLE;
  }
}
