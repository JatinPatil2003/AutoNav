import { IRobotDocument, RobotStatus } from "./robot.model";

export interface CreateRobotInput {
  name: string;
  vpnHostname: string;
  displayName?: string;
}

export interface IRobotRepository {
  findById(id: string): Promise<IRobotDocument | null>;
  findByName(name: string): Promise<IRobotDocument | null>;
  listByStatus(status: RobotStatus): Promise<IRobotDocument[]>;
  listAll(): Promise<IRobotDocument[]>;
  create(input: CreateRobotInput): Promise<IRobotDocument>;
  assignToUser(robotId: string, userId: string): Promise<IRobotDocument | null>;
}
