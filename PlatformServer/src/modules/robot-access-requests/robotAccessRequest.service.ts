import { ConflictError } from "../../common/errors";
import { IUserRepository } from "../users/user.repository.interface";
import { IRobotAccessRequestRepository } from "./robotAccessRequest.repository.interface";
import { RobotAccessRequestStatus } from "./robotAccessRequest.model";
import { RobotAccessRequestDto, toRobotAccessRequestDto } from "./robotAccessRequest.dto";

const DUPLICATE_KEY_ERROR_CODE = 11000;

export class RobotAccessRequestService {
  constructor(
    private readonly requestRepository: IRobotAccessRequestRepository,
    private readonly userRepository: IUserRepository,
  ) {}

  async createRequest(userId: string): Promise<RobotAccessRequestDto> {
    const user = await this.userRepository.findById(userId);

    if (user?.robot) {
      throw new ConflictError("You already have a robot assigned");
    }

    try {
      const request = await this.requestRepository.create(userId);
      return toRobotAccessRequestDto(request);
    } catch (err) {
      if (isDuplicateKeyError(err)) {
        throw new ConflictError("You already have a pending robot access request");
      }
      throw err;
    }
  }

  async getMyLatestRequest(userId: string): Promise<RobotAccessRequestDto | null> {
    const request = await this.requestRepository.findLatestForUser(userId);
    return request ? toRobotAccessRequestDto(request) : null;
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
