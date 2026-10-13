import { ConflictError, ForbiddenError, NotFoundError } from "../../common/errors";
import { logger } from "../../common/utils/logger.util";
import { IRobotAccessRequestRepository } from "../robot-access-requests/robotAccessRequest.repository.interface";
import { IRobotRepository } from "../robots/robot.repository.interface";
import { IUserRepository } from "../users/user.repository.interface";
import { UserRole } from "../users/user.model";
import { PublicUserDto, toPublicUserDto } from "../users/user.dto";
import {
  RobotAccessRequestDto,
  toRobotAccessRequestDto,
} from "../robot-access-requests/robotAccessRequest.dto";

/**
 * Orchestrates the robot-access-request approval workflow, which spans three
 * modules (request, robot, user). A single Mongo replica-set transaction
 * would be the ideal consistency boundary here; this milestone instead
 * performs the three updates sequentially (acceptable for a low-volume,
 * admin-only operation) and can be upgraded to a session-scoped transaction
 * once the deployment target is a replica set.
 */
export class AdminService {
  constructor(
    private readonly requestRepository: IRobotAccessRequestRepository,
    private readonly robotRepository: IRobotRepository,
    private readonly userRepository: IUserRepository,
  ) {}

  async approveRequest(
    requestId: string,
    robotId: string,
    adminUserId: string,
  ): Promise<RobotAccessRequestDto> {
    const request = await this.requestRepository.findById(requestId);
    if (!request) throw new NotFoundError("Robot access request not found");
    if (request.status !== "pending") {
      throw new ConflictError(`Request is already ${request.status}`);
    }

    const robot = await this.robotRepository.findById(robotId);
    if (!robot) throw new NotFoundError("Robot not found");
    if (robot.status !== "unassigned") {
      throw new ConflictError("Robot is already assigned to another user");
    }

    await this.robotRepository.assignToUser(robotId, request.user.toString());
    await this.userRepository.assignRobot(request.user.toString(), robotId);
    const updatedRequest = await this.requestRepository.markApproved(requestId, adminUserId);
    logger.debug(
      { requestId, robotId, userId: request.user.toString(), adminUserId },
      "admin.approveRequest: robot assigned",
    );

    return toRobotAccessRequestDto(updatedRequest!);
  }

  async rejectRequest(
    requestId: string,
    adminUserId: string,
    reason?: string,
  ): Promise<RobotAccessRequestDto> {
    const request = await this.requestRepository.findById(requestId);
    if (!request) throw new NotFoundError("Robot access request not found");
    if (request.status !== "pending") {
      throw new ConflictError(`Request is already ${request.status}`);
    }

    const updatedRequest = await this.requestRepository.markRejected(requestId, adminUserId, reason);
    logger.debug({ requestId, adminUserId, reason }, "admin.rejectRequest: request rejected");
    return toRobotAccessRequestDto(updatedRequest!);
  }

  async listUsers(): Promise<PublicUserDto[]> {
    const users = await this.userRepository.listAllPopulated();
    return users.map(toPublicUserDto);
  }

  async updateUserRole(
    targetUserId: string,
    role: UserRole,
    actingAdminId: string,
  ): Promise<PublicUserDto> {
    if (targetUserId === actingAdminId) {
      throw new ForbiddenError("You cannot change your own role");
    }

    const user = await this.userRepository.updateRole(targetUserId, role);
    if (!user) throw new NotFoundError("User not found");

    logger.debug({ targetUserId, role, actingAdminId }, "admin.updateUserRole: role changed");
    return toPublicUserDto(user);
  }
}
