import { ConflictError, NotFoundError } from "../../common/errors";
import { IRobotAssignmentRepository } from "./robotAssignment.repository.interface";
import { IRobotRepository } from "../robots/robot.repository.interface";
import { PublicRobotDto } from "../users/user.dto";
import { RobotAssignmentDto, toRobotAssignmentDto } from "./robotAssignment.dto";
import { IRobotDocument } from "../robots/robot.model";
import { IRobotNotifier } from "./robotNotifier.interface";

export class RobotAssignmentService {
  constructor(
    private readonly assignmentRepository: IRobotAssignmentRepository,
    private readonly robotRepository: IRobotRepository,
    private readonly robotNotifier: IRobotNotifier,
  ) {}

  async grant(
    userId: string,
    robotId: string,
    grantedBy: string,
    access?: { hmiAccess?: boolean; platformAccess?: boolean },
  ): Promise<RobotAssignmentDto> {
    const robot = await this.robotRepository.findById(robotId);
    if (!robot) throw new NotFoundError("Robot not found");

    const existingActive = await this.assignmentRepository.findActiveByUserAndRobot(userId, robotId);
    if (existingActive) {
      throw new ConflictError("This user already has active access to this robot");
    }

    const assignment = await this.assignmentRepository.upsertActive(userId, robotId, grantedBy, access);
    return toRobotAssignmentDto(assignment);
  }

  /** Flips hmiAccess/platformAccess on an existing assignment without touching status or PIN. */
  async updateAccess(
    assignmentId: string,
    access: { hmiAccess?: boolean; platformAccess?: boolean },
  ): Promise<RobotAssignmentDto> {
    const assignment = await this.assignmentRepository.updateAccess(assignmentId, access);
    if (!assignment) throw new NotFoundError("Robot assignment not found");

    // hmiAccess revocation must take effect on the robot immediately, same
    // as a full revoke — otherwise it only self-corrects on the next sync.
    if (access.hmiAccess === false) {
      const robot = await this.robotRepository.findById(assignment.robot.toString());
      if (robot) {
        void this.robotNotifier.notifyRosterInvalidate(robot.robotId).catch(() => undefined);
      }
    }

    return toRobotAssignmentDto(assignment);
  }

  async revoke(assignmentId: string, revokedBy: string, reason?: string): Promise<RobotAssignmentDto> {
    const assignment = await this.assignmentRepository.revoke(assignmentId, revokedBy, reason);
    if (!assignment) throw new NotFoundError("Robot assignment not found");

    const robot = await this.robotRepository.findById(assignment.robot.toString());
    if (robot) {
      // Best-effort — if the robot is offline it self-corrects on its next
      // boot/periodic roster resync (see RobotServer's sync job).
      void this.robotNotifier.notifyRosterInvalidate(robot.robotId).catch(() => undefined);
    }

    return toRobotAssignmentDto(assignment);
  }

  async listActiveRobotsForUser(userId: string): Promise<PublicRobotDto[]> {
    const assignments = await this.assignmentRepository.listActiveForUser(userId);
    return assignments
      .map((a) => a.robot as unknown as IRobotDocument)
      .filter((robot) => !!robot && typeof robot === "object" && "name" in robot)
      .map((robot) => ({
        id: robot._id.toString(),
        name: robot.name,
        displayName: robot.displayName,
        vpnHostname: robot.vpnHostname,
        status: robot.status,
      }));
  }

  async listForRobot(robotId: string): Promise<RobotAssignmentDto[]> {
    const assignments = await this.assignmentRepository.listForRobot(robotId);
    return assignments.map(toRobotAssignmentDto);
  }

  async listForUser(userId: string): Promise<RobotAssignmentDto[]> {
    const assignments = await this.assignmentRepository.listForUser(userId);
    return assignments.map(toRobotAssignmentDto);
  }

  async deleteAllForUser(userId: string): Promise<void> {
    await this.assignmentRepository.deleteForUser(userId);
  }

  async hasActiveAssignment(userId: string, robotId: string): Promise<boolean> {
    const assignment = await this.assignmentRepository.findActiveByUserAndRobot(userId, robotId);
    return !!assignment;
  }
}
