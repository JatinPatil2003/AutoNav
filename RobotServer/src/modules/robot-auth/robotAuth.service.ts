import { UnauthorizedError } from "../../common/errors";
import { config } from "../../config/env";
import { comparePin } from "../../common/utils/pin.util";
import { signSessionAccessToken, signSessionRefreshToken, verifySessionRefreshToken } from "../../common/utils/jwt.util";
import { RobotSessionTokenPair } from "../../common/types/auth.types";
import { IPlatformClient } from "../../clients/platformClient.interface";
import { DeviceIdentityService } from "../../ros/deviceIdentity";
import { ILocalUserRepository } from "./localUser.repository.interface";
import { logger } from "../../common/utils/logger.util";

export interface IdentifyResult {
  /**
   * "has-access": found in this robot's local roster, proceed to PIN
   * login/setup. "no-account": no PlatformServer account exists for this
   * email at all. "no-access": the account exists but isn't assigned to
   * this robot — different user-facing messages for each.
   */
  status: "has-access" | "no-account" | "no-access";
  pinSet: boolean;
  name?: string;
  avatarUrl?: string;
}

export interface RosterUserSummary {
  email: string;
  name: string;
  avatarUrl?: string;
  pinSet: boolean;
}

export class RobotAuthService {
  constructor(
    private readonly localUserRepository: ILocalUserRepository,
    private readonly platformClient: IPlatformClient,
    private readonly deviceIdentity: DeviceIdentityService,
  ) {}

  /** The lock-screen roster — who can log into this robot right now. */
  async listActiveUsers(): Promise<RosterUserSummary[]> {
    const robotId = this.deviceIdentity.getRobotId();
    if (!robotId) return [];

    const users = await this.localUserRepository.listActive(robotId);
    return users.map((u) => ({ email: u.email, name: u.name, avatarUrl: u.avatarUrl, pinSet: !!u.pinHash }));
  }

  async identify(email: string): Promise<IdentifyResult> {
    const robotId = this.deviceIdentity.getRobotId();
    const user = robotId ? await this.localUserRepository.findByEmail(email, robotId) : null;
    if (user && user.status === "active") {
      return { status: "has-access", pinSet: !!user.pinHash, name: user.name, avatarUrl: user.avatarUrl };
    }

    const accountExists = await this.platformClient.checkUserExists(email);
    return { status: accountExists ? "no-access" : "no-account", pinSet: false };
  }

  async loginWithPin(email: string, pin: string): Promise<RobotSessionTokenPair> {
    const robotId = this.deviceIdentity.getRobotId();
    const user = robotId ? await this.localUserRepository.findByEmail(email, robotId) : null;
    if (!user || user.status !== "active") {
      throw new UnauthorizedError("Invalid email or PIN");
    }

    if (user.lockedUntil && user.lockedUntil.getTime() > Date.now()) {
      throw new UnauthorizedError("Too many attempts. Please try again later.");
    }

    if (!user.pinHash) {
      throw new UnauthorizedError("No PIN has been set up for this user on this robot yet");
    }

    const isValid = await comparePin(pin, user.pinHash);
    if (!isValid) {
      await this.localUserRepository.recordFailedAttempt(
        user._id.toString(),
        config.pinLogin.maxAttempts,
        config.pinLogin.lockoutMinutes,
      );
      logger.debug({ userId: user.platformUserId }, "robotAuth.loginWithPin: rejected, bad pin");
      throw new UnauthorizedError("Invalid email or PIN");
    }

    await this.localUserRepository.resetAttempts(user._id.toString());
    logger.debug({ userId: user.platformUserId }, "robotAuth.loginWithPin: success");
    return this.issueSessionTokens(user.platformUserId, user.email, user.name, user.avatarUrl);
  }

  async refresh(refreshToken: string): Promise<RobotSessionTokenPair> {
    let payload;
    try {
      payload = verifySessionRefreshToken(refreshToken);
    } catch {
      throw new UnauthorizedError("Invalid or expired refresh token");
    }
    return this.issueSessionTokens(payload.sub, payload.email, payload.name, payload.avatarUrl);
  }

  async requestPinSetupOtp(email: string): Promise<void> {
    await this.platformClient.requestPinSetupOtp(email);
  }

  async verifyPinSetupOtp(email: string, otp: string): Promise<{ pinSetupToken: string }> {
    return this.platformClient.verifyPinSetupOtp(email, otp);
  }

  /**
   * Caches the new PIN locally but deliberately does NOT issue a session —
   * the user must log in with the PIN they just set, same as any other
   * returning user, rather than being silently signed in.
   */
  async confirmPinSetup(pinSetupToken: string, pin: string): Promise<{ email: string }> {
    const entry = await this.platformClient.confirmPinSetup(pinSetupToken, pin);
    const robotId = this.deviceIdentity.getRobotId();
    if (robotId) {
      await this.localUserRepository.applyEntry(entry, robotId);
    }
    logger.debug({ userId: entry.userId }, "robotAuth.confirmPinSetup: pin cached locally");
    return { email: entry.email };
  }

  private issueSessionTokens(
    userId: string,
    email: string,
    name: string,
    avatarUrl?: string,
  ): RobotSessionTokenPair {
    return {
      accessToken: signSessionAccessToken({
        sub: userId,
        email,
        name,
        avatarUrl,
        type: "robot-session-access",
      }),
      refreshToken: signSessionRefreshToken({
        sub: userId,
        email,
        name,
        avatarUrl,
        type: "robot-session-refresh",
      }),
    };
  }
}
