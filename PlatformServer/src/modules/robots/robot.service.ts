import crypto from "node:crypto";
import { ConflictError, NotFoundError } from "../../common/errors";
import { IRobotRepository } from "./robot.repository.interface";
import { RobotStatus } from "./robot.model";
import { RobotDto, RobotSummaryDto, toRobotDto, toRobotSummaryDto } from "./robot.dto";
import { CreateRobotDto } from "./robot.validation";
import { IRobotAssignmentRepository } from "../robot-assignments/robotAssignment.repository.interface";
import { IRobotAccessRequestRepository } from "../robot-access-requests/robotAccessRequest.repository.interface";

/** Hex length of the bearer secret handed to a robot at provisioning/rotation (32 random bytes). */
const LINK_SECRET_BYTES = 32;

function hashLinkSecret(secret: string): string {
  return crypto.createHash("sha256").update(secret).digest("hex");
}

export class RobotService {
  constructor(
    private readonly robotRepository: IRobotRepository,
    private readonly robotAssignmentRepository: IRobotAssignmentRepository,
    private readonly requestRepository: IRobotAccessRequestRepository,
  ) {}

  /** Returns the created robot plus the plaintext robot-link secret — shown exactly once. */
  async createRobot(input: CreateRobotDto): Promise<{ robot: RobotDto; linkSecret: string }> {
    const [existingByName, existingByRobotId] = await Promise.all([
      this.robotRepository.findByName(input.name),
      this.robotRepository.findByRobotId(input.robotId),
    ]);
    if (existingByName) {
      throw new ConflictError(`Robot with name "${input.name}" already exists`);
    }
    if (existingByRobotId) {
      throw new ConflictError(`Robot with robotId "${input.robotId}" already exists`);
    }

    const robot = await this.robotRepository.create(input);
    const linkSecret = crypto.randomBytes(LINK_SECRET_BYTES).toString("hex");
    await this.robotRepository.setLinkSecretHash(robot._id.toString(), hashLinkSecret(linkSecret));

    return { robot: toRobotDto(robot), linkSecret };
  }

  /** Rotates a robot's link secret, invalidating its current persistent connection going forward. */
  async rotateLinkSecret(robotId: string): Promise<{ linkSecret: string }> {
    const robot = await this.robotRepository.findById(robotId);
    if (!robot) throw new NotFoundError("Robot not found");

    const linkSecret = crypto.randomBytes(LINK_SECRET_BYTES).toString("hex");
    await this.robotRepository.setLinkSecretHash(robot._id.toString(), hashLinkSecret(linkSecret));

    return { linkSecret };
  }

  async listByStatus(status: RobotStatus): Promise<RobotDto[]> {
    const robots = await this.robotRepository.listByStatus(status);
    return robots.map(toRobotDto);
  }

  async listAll(): Promise<RobotDto[]> {
    const robots = await this.robotRepository.listAll();
    return robots.map(toRobotDto);
  }

  /** Non-admin-safe listing (no vpnHostname/status/lastSeenAt) for the request-access picker. */
  async listAllSummaries(): Promise<RobotSummaryDto[]> {
    const robots = await this.robotRepository.listAll();
    return robots.filter((r) => r.status !== "retired").map(toRobotSummaryDto);
  }

  /** Permanently removes a robot along with its assignments and access requests. */
  async deleteRobot(id: string): Promise<void> {
    const robot = await this.robotRepository.findById(id);
    if (!robot) throw new NotFoundError("Robot not found");

    await Promise.all([
      this.robotAssignmentRepository.deleteForRobot(id),
      this.requestRepository.deleteForRobot(id),
    ]);
    await this.robotRepository.deleteById(id);
  }
}
