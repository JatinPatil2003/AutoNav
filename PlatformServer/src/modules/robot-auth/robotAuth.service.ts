import { NotFoundError, UnauthorizedError } from "../../common/errors";
import { logger } from "../../common/utils/logger.util";
import { hashPassword } from "../../common/utils/password.util";
import { compareOtp, generateOtp, hashOtp } from "../../common/utils/otp.util";
import { signPinSetupToken, verifyPinSetupToken } from "../../common/utils/jwt.util";
import { config } from "../../config/env";
import { IRobotAssignmentRepository } from "../robot-assignments/robotAssignment.repository.interface";
import { IUserRepository } from "../users/user.repository.interface";
import { IEmailOtpRepository } from "../auth/emailOtp.repository.interface";
import { IEmailOtpSender } from "../auth/otpSender.interface";
import { RosterEntryDto, toRosterEntryDto } from "./robotAuth.dto";

export class RobotAuthService {
  constructor(
    private readonly robotAssignmentRepository: IRobotAssignmentRepository,
    private readonly userRepository: IUserRepository,
    private readonly otpRepository: IEmailOtpRepository,
    private readonly emailOtpSender: IEmailOtpSender,
  ) {}

  /**
   * Deliberately NOT anti-enumeration like forgotPassword — RobotApp needs
   * to tell "no such account, go sign up" apart from "account exists but
   * isn't assigned to this robot, contact your admin", and this is gated
   * behind requireRobotAuth (a provisioned robot's own credential), not a
   * public endpoint, so the exposure is to robots we already trust.
   */
  async checkUserExists(email: string): Promise<boolean> {
    const user = await this.userRepository.findByEmail(email);
    return !!user;
  }

  /** Scoped to `robotMongoId` only — the caller's own robot, never a client-supplied id. */
  async getRoster(robotMongoId: string): Promise<RosterEntryDto[]> {
    const assignments = await this.robotAssignmentRepository.listActiveForRobotWithPin(robotMongoId);
    return assignments
      .filter((a) => a.user && typeof a.user === "object" && "email" in a.user)
      .map((a) =>
        toRosterEntryDto(
          a,
          a.user as unknown as {
            _id: { toString(): string };
            email: string;
            name: string;
            avatarUrl?: string;
          },
        ),
      );
  }

  /**
   * Always resolves successfully whether or not the email/assignment exists,
   * so this can't be used to enumerate accounts or robot rosters — same
   * anti-enumeration pattern as AuthService.forgotPassword.
   */
  async requestPinSetupOtp(robotMongoId: string, email: string): Promise<void> {
    const user = await this.userRepository.findByEmail(email);
    if (!user) return;

    const assignment = await this.robotAssignmentRepository.findActiveByUserAndRobot(
      user._id.toString(),
      robotMongoId,
    );
    if (!assignment) return;

    await this.otpRepository.invalidateActiveForUser(user._id.toString(), "robot-pin-setup");

    const otp = generateOtp();
    const otpHash = await hashOtp(otp);
    const expiresAt = new Date(Date.now() + config.otp.expiryMinutes * 60 * 1000);
    await this.otpRepository.create(user._id.toString(), "robot-pin-setup", otpHash, expiresAt);

    await this.emailOtpSender.sendOtp(user.email, otp);
    logger.debug({ userId: user._id.toString() }, "robotAuth.requestPinSetupOtp: OTP issued");
  }

  async verifyPinSetupOtp(email: string, otp: string, callerRobotId: string): Promise<{ pinSetupToken: string }> {
    const user = await this.userRepository.findByEmail(email);
    if (!user) {
      throw new UnauthorizedError("Invalid or expired code");
    }

    const otpRecord = await this.otpRepository.findLatestActiveForUser(
      user._id.toString(),
      "robot-pin-setup",
    );
    if (!otpRecord) {
      throw new UnauthorizedError("Invalid or expired code");
    }

    const isValid = await compareOtp(otp, otpRecord.otpHash);
    if (!isValid) {
      throw new UnauthorizedError("Invalid or expired code");
    }

    await this.otpRepository.markConsumed(otpRecord._id.toString());

    return { pinSetupToken: signPinSetupToken(user._id.toString(), callerRobotId) };
  }

  /**
   * `callerRobotId` is the robot slug making this call (from req.robot,
   * i.e. its own verified service-auth identity) — must match the
   * pinSetupToken's bound robotId, so a token minted for one robot can
   * never be replayed to set a PIN on a different robot.
   */
  async confirmPinSetup(
    pinSetupToken: string,
    pin: string,
    callerRobotId: string,
    robotMongoId: string,
  ): Promise<RosterEntryDto> {
    let payload;
    try {
      payload = verifyPinSetupToken(pinSetupToken);
    } catch {
      throw new UnauthorizedError("Invalid or expired pin setup token");
    }

    if (payload.robotId !== callerRobotId) {
      throw new UnauthorizedError("Pin setup token is not valid for this robot");
    }

    const assignment = await this.robotAssignmentRepository.findActiveByUserAndRobot(
      payload.sub,
      robotMongoId,
    );
    if (!assignment) {
      throw new NotFoundError("Robot assignment not found");
    }

    const pinHash = await hashPassword(pin);
    const updated = await this.robotAssignmentRepository.setPin(assignment._id.toString(), pinHash);

    const user = await this.userRepository.findById(payload.sub);
    if (!updated || !user) {
      throw new NotFoundError("Robot assignment not found");
    }

    logger.debug({ userId: payload.sub, robotId: callerRobotId }, "robotAuth.confirmPinSetup: pin set");
    return toRosterEntryDto(updated, user);
  }
}
