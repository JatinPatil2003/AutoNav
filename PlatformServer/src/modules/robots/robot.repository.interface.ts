import { IEncryptedSecret, IRobotDocument, RobotStatus } from "./robot.model";

export interface CreateRobotInput {
  robotId: string;
  name: string;
  vpnHostname: string;
  displayName?: string;
  robotSecretEncrypted: IEncryptedSecret;
  robotSecretHash: string;
}

export interface IRobotRepository {
  findById(id: string): Promise<IRobotDocument | null>;
  findByName(name: string): Promise<IRobotDocument | null>;
  findByRobotId(robotId: string): Promise<IRobotDocument | null>;
  /** Identifies a robot purely from its secret — see requireRobotAuth.middleware.ts. */
  findBySecretHash(robotSecretHash: string): Promise<IRobotDocument | null>;
  listByStatus(status: RobotStatus): Promise<IRobotDocument[]>;
  listAll(): Promise<IRobotDocument[]>;
  create(input: CreateRobotInput): Promise<IRobotDocument>;
  rotateSecret(
    id: string,
    robotSecretEncrypted: IEncryptedSecret,
    robotSecretHash: string,
  ): Promise<IRobotDocument | null>;
  markSeen(robotId: string): Promise<void>;
  deleteById(id: string): Promise<void>;
}
