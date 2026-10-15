import { NotFoundError } from "../../common/errors";
import { IUserRepository } from "./user.repository.interface";
import { PublicUserDto, toPublicUserDto } from "./user.dto";
import { RobotAssignmentService } from "../robot-assignments/robotAssignment.service";

export class UserService {
  constructor(
    private readonly userRepository: IUserRepository,
    private readonly robotAssignmentService: RobotAssignmentService,
  ) {}

  async getMe(userId: string): Promise<PublicUserDto> {
    const user = await this.userRepository.findByIdPopulated(userId);

    if (!user) {
      throw new NotFoundError("User not found");
    }

    const robots = await this.robotAssignmentService.listActiveRobotsForUser(userId);
    return toPublicUserDto(user, robots);
  }
}
