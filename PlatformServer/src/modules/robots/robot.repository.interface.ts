import { IRobotDocument, RobotStatus } from "./robot.model";

export interface CreateRobotInput {
  robotId: string;
  name: string;
  vpnHostname: string;
  displayName?: string;
}

export interface IRobotRepository {
  findById(id: string): Promise<IRobotDocument | null>;
  findByName(name: string): Promise<IRobotDocument | null>;
  /** Identifies a robot purely from its own device id — see resolveRobot.middleware.ts. */
  findByRobotId(robotId: string): Promise<IRobotDocument | null>;
  /** Same lookup as findByRobotId but with the normally-hidden linkSecretHash selected — used only by the robot-link authenticator. */
  findByRobotIdWithLinkSecret(robotId: string): Promise<IRobotDocument | null>;
  listByStatus(status: RobotStatus): Promise<IRobotDocument[]>;
  listAll(): Promise<IRobotDocument[]>;
  create(input: CreateRobotInput): Promise<IRobotDocument>;
  markSeen(robotId: string): Promise<void>;
  /** Updates lastSeenAt only, without flipping status — used on robot-link disconnect. */
  touchLastSeen(robotId: string): Promise<void>;
  deleteById(id: string): Promise<void>;
  /** Sets (or rotates) the robot-link secret hash. Called once at provisioning and again on rotation. */
  setLinkSecretHash(id: string, linkSecretHash: string): Promise<void>;
}
