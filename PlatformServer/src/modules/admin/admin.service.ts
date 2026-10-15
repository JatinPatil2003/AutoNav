import { ConflictError, ForbiddenError, NotFoundError } from "../../common/errors";
import { logger } from "../../common/utils/logger.util";
import { IRobotAccessRequestRepository } from "../robot-access-requests/robotAccessRequest.repository.interface";
import { IUserRepository } from "../users/user.repository.interface";
import { UserRole } from "../users/user.model";
import { PublicUserDto, toPublicUserDto } from "../users/user.dto";
import { RobotAssignmentService } from "../robot-assignments/robotAssignment.service";
import { RobotAssignmentDto } from "../robot-assignments/robotAssignment.dto";
import {
  RobotAccessRequestDto,
  toRobotAccessRequestDto,
} from "../robot-access-requests/robotAccessRequest.dto";

export class AdminService {
  constructor(
    private readonly requestRepository: IRobotAccessRequestRepository,
    private readonly userRepository: IUserRepository,
    private readonly robotAssignmentService: RobotAssignmentService,
  ) {}

  /**
   * Grants access to the originally requested robot by default; an explicit
   * `robotIdOverride` lets the admin redirect the grant to a different robot.
   */
  async approveRequest(
    requestId: string,
    adminUserId: string,
    robotIdOverride?: string,
  ): Promise<RobotAccessRequestDto> {
    const request = await this.requestRepository.findById(requestId);
    if (!request) throw new NotFoundError("Robot access request not found");
    if (request.status !== "pending") {
      throw new ConflictError(`Request is already ${request.status}`);
    }

    const robotId = robotIdOverride ?? request.robot.toString();
    await this.robotAssignmentService.grant(request.user.toString(), robotId, adminUserId);
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

  /** Direct grant, bypassing the request/approval workflow. */
  async grantRobotAssignment(
    userId: string,
    robotId: string,
    adminUserId: string,
    access?: { hmiAccess?: boolean; platformAccess?: boolean },
  ): Promise<RobotAssignmentDto> {
    return this.robotAssignmentService.grant(userId, robotId, adminUserId, access);
  }

  async revokeRobotAssignment(
    assignmentId: string,
    adminUserId: string,
    reason?: string,
  ): Promise<RobotAssignmentDto> {
    return this.robotAssignmentService.revoke(assignmentId, adminUserId, reason);
  }

  async updateRobotAssignmentAccess(
    assignmentId: string,
    access: { hmiAccess?: boolean; platformAccess?: boolean },
  ): Promise<RobotAssignmentDto> {
    return this.robotAssignmentService.updateAccess(assignmentId, access);
  }

  async listAssignmentsForRobot(robotId: string): Promise<RobotAssignmentDto[]> {
    return this.robotAssignmentService.listForRobot(robotId);
  }

  async listAssignmentsForUser(userId: string): Promise<RobotAssignmentDto[]> {
    return this.robotAssignmentService.listForUser(userId);
  }

  async listUsers(): Promise<PublicUserDto[]> {
    const users = await this.userRepository.listAllPopulated();
    return Promise.all(
      users.map(async (user) => {
        const robots = await this.robotAssignmentService.listActiveRobotsForUser(user._id.toString());
        return toPublicUserDto(user, robots);
      }),
    );
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
    const robots = await this.robotAssignmentService.listActiveRobotsForUser(targetUserId);
    return toPublicUserDto(user, robots);
  }

  async deleteUser(targetUserId: string, actingAdminId: string): Promise<void> {
    if (targetUserId === actingAdminId) {
      throw new ForbiddenError("You cannot delete your own account");
    }

    const user = await this.userRepository.findById(targetUserId);
    if (!user) throw new NotFoundError("User not found");

    await Promise.all([
      this.robotAssignmentService.deleteAllForUser(targetUserId),
      this.requestRepository.deleteForUser(targetUserId),
    ]);
    await this.userRepository.deleteById(targetUserId);

    logger.debug({ targetUserId, actingAdminId }, "admin.deleteUser: user deleted");
  }
}
