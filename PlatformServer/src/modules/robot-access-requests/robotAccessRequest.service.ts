import { ConflictError, NotFoundError } from "../../common/errors";
import { IRobotRepository } from "../robots/robot.repository.interface";
import { RobotAssignmentService } from "../robot-assignments/robotAssignment.service";
import { IRobotAccessRequestRepository } from "./robotAccessRequest.repository.interface";
import { RobotAccessRequestStatus } from "./robotAccessRequest.model";
import { RobotAccessRequestDto, toRobotAccessRequestDto } from "./robotAccessRequest.dto";

const DUPLICATE_KEY_ERROR_CODE = 11000;

export class RobotAccessRequestService {
  constructor(
    private readonly requestRepository: IRobotAccessRequestRepository,
    private readonly robotRepository: IRobotRepository,
    private readonly robotAssignmentService: RobotAssignmentService,
  ) {}

  async createRequest(userId: string, robotId: string): Promise<RobotAccessRequestDto> {
    const robot = await this.robotRepository.findById(robotId);
    if (!robot) {
      throw new NotFoundError("Robot not found");
    }

    const hasAccess = await this.robotAssignmentService.hasActiveAssignment(userId, robotId);
    if (hasAccess) {
      throw new ConflictError("You already have access to this robot");
    }

    try {
      const request = await this.requestRepository.create(userId, robotId);
      return toRobotAccessRequestDto(request);
    } catch (err) {
      if (isDuplicateKeyError(err)) {
        throw new ConflictError("You already have a pending request for this robot");
      }
      throw err;
    }
  }

  async listMine(userId: string): Promise<RobotAccessRequestDto[]> {
    const requests = await this.requestRepository.listForUser(userId);
    return requests.map(toRobotAccessRequestDto);
  }

  async listByStatus(status: RobotAccessRequestStatus): Promise<RobotAccessRequestDto[]> {
    const requests = await this.requestRepository.listByStatus(status);
    return requests.map(toRobotAccessRequestDto);
  }
}

function isDuplicateKeyError(err: unknown): boolean {
  return (
    typeof err === "object" &&
    err !== null &&
    "code" in err &&
    (err as { code?: number }).code === DUPLICATE_KEY_ERROR_CODE
  );
}
