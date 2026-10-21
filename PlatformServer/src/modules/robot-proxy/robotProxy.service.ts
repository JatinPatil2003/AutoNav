import { NotFoundError } from "../../common/errors";
import { IRobotRepository } from "../robots/robot.repository.interface";
import { IRobotApiClient } from "./robotApiClient.interface";

/** Mirrors RobotServer's SlamBackend (mapping.types.ts) — contract by convention, not a shared package, same as the robot-link protocol. */
export type SlamBackend = "cartographer" | "slam_toolbox";

export interface Pose {
  x: number;
  y: number;
  theta: number;
}

export interface Goal extends Pose {
  mapName: string;
  name: string;
}

/**
 * The request/response half of Phase 3's dashboard integration: proxies
 * navigation/mapping commands straight to the robot's own RobotServer REST
 * API over the VPN (HttpRobotApiClient), rather than relaying them over the
 * robot-link WebSocket — that socket is reserved for streaming (pose/map/
 * nav-status, see telemetry-relay). Every method here is a thin pass-
 * through; the one shared piece of logic is resolving robotMongoId to the
 * robot's vpnHostname and failing with a clear 404 if it doesn't exist.
 */
export class RobotProxyService {
  constructor(
    private readonly robotRepository: IRobotRepository,
    private readonly apiClient: IRobotApiClient,
  ) {}

  async startMapping(robotMongoId: string, backend?: SlamBackend): Promise<void> {
    const robot = await this.requireRobot(robotMongoId);
    await this.apiClient.request(robot, "POST", "/api/mapping/start", backend ? { backend } : {});
  }

  async stopMapping(robotMongoId: string): Promise<void> {
    const robot = await this.requireRobot(robotMongoId);
    await this.apiClient.request(robot, "POST", "/api/mapping/stop");
  }

  async captureWaypoint(robotMongoId: string, name: string): Promise<void> {
    const robot = await this.requireRobot(robotMongoId);
    await this.apiClient.request(robot, "POST", "/api/mapping/waypoints", { name });
  }

  async commitWaypoints(robotMongoId: string, mapName: string): Promise<{ processed: number }> {
    const robot = await this.requireRobot(robotMongoId);
    return this.apiClient.request(robot, "POST", "/api/mapping/waypoints/commit", { mapName });
  }

  async saveMap(robotMongoId: string, mapName: string): Promise<void> {
    const robot = await this.requireRobot(robotMongoId);
    await this.apiClient.request(robot, "POST", "/api/mapping/save", { mapName });
  }

  async listMaps(robotMongoId: string): Promise<{ maps: string[] }> {
    const robot = await this.requireRobot(robotMongoId);
    return this.apiClient.request(robot, "GET", "/api/navigation/maps");
  }

  async selectMap(robotMongoId: string, mapName: string): Promise<void> {
    const robot = await this.requireRobot(robotMongoId);
    await this.apiClient.request(robot, "POST", `/api/navigation/maps/${encodeURIComponent(mapName)}/select`);
  }

  async listGoals(robotMongoId: string, mapName: string): Promise<unknown> {
    const robot = await this.requireRobot(robotMongoId);
    return this.apiClient.request(robot, "GET", `/api/navigation/maps/${encodeURIComponent(mapName)}/goals`);
  }

  async saveGoal(robotMongoId: string, goal: Goal): Promise<void> {
    const robot = await this.requireRobot(robotMongoId);
    await this.apiClient.request(robot, "POST", "/api/navigation/goals", goal);
  }

  async startGoal(robotMongoId: string, mapName: string, name: string): Promise<void> {
    const robot = await this.requireRobot(robotMongoId);
    await this.apiClient.request(robot, "POST", "/api/navigation/goal/start", { mapName, name });
  }

  async cancelGoal(robotMongoId: string): Promise<void> {
    const robot = await this.requireRobot(robotMongoId);
    await this.apiClient.request(robot, "POST", "/api/navigation/goal/cancel");
  }

  private async requireRobot(robotMongoId: string) {
    const robot = await this.robotRepository.findById(robotMongoId);
    if (!robot) throw new NotFoundError("Robot not found");
    return robot;
  }
}
