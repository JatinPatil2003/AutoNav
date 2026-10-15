import { ConflictError, NotFoundError } from "../../common/errors";
import { IRobotRepository } from "./robot.repository.interface";
import { RobotStatus } from "./robot.model";
import { RobotDto, RobotSummaryDto, toRobotDto, toRobotSummaryDto } from "./robot.dto";
import { CreateRobotDto } from "./robot.validation";
import { IRobotAssignmentRepository } from "../robot-assignments/robotAssignment.repository.interface";
import { IRobotAccessRequestRepository } from "../robot-access-requests/robotAccessRequest.repository.interface";

export class RobotService {
  constructor(
    private readonly robotRepository: IRobotRepository,
    private readonly robotAssignmentRepository: IRobotAssignmentRepository,
    private readonly requestRepository: IRobotAccessRequestRepository,
  ) {}

  async createRobot(input: CreateRobotDto): Promise<RobotDto> {
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
    return toRobotDto(robot);
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
