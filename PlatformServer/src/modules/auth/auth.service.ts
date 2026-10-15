import { ConflictError, UnauthorizedError } from "../../common/errors";
import { TokenPair } from "../../common/types/auth.types";
import { logger } from "../../common/utils/logger.util";
import { comparePassword, hashPassword } from "../../common/utils/password.util";
import { compareOtp, generateOtp, hashOtp } from "../../common/utils/otp.util";
import {
  signAccessToken,
  signRefreshToken,
  signResetToken,
  verifyRefreshToken,
  verifyResetToken,
} from "../../common/utils/jwt.util";
import { config } from "../../config/env";
import { GoogleProfile } from "../../config/passport";
import { IUserDocument, UserRole } from "../users/user.model";
import { IUserRepository } from "../users/user.repository.interface";
import { PublicUserDto, toPublicUserDto } from "../users/user.dto";
import { NotFoundError } from "../../common/errors";
import {
  ForgotPasswordDto,
  LoginDto,
  ResetPasswordDto,
  SetPasswordDto,
  SignupDto,
  VerifyOtpDto,
} from "./auth.validation";
import { IEmailOtpRepository } from "./emailOtp.repository.interface";
import { IEmailOtpSender } from "./otpSender.interface";

export class AuthService {
  constructor(
    private readonly userRepository: IUserRepository,
    private readonly otpRepository: IEmailOtpRepository,
    private readonly emailOtpSender: IEmailOtpSender,
  ) {}

  async signup(input: SignupDto): Promise<{ user: PublicUserDto; tokens: TokenPair }> {
    logger.debug({ email: input.email }, "auth.signup: attempt");
    const existing = await this.userRepository.findByEmail(input.email);
    if (existing) {
      logger.debug({ email: input.email }, "auth.signup: rejected, email already exists");
      throw new ConflictError("An account with this email already exists");
    }

    const passwordHash = await hashPassword(input.password);
    const user = await this.userRepository.create({
      email: input.email,
      name: input.name,
      role: this.resolveRole(input.email),
      authProviders: ["local"],
      passwordHash,
    });
    logger.debug({ userId: user._id.toString(), role: user.role }, "auth.signup: account created");

    return { user: toPublicUserDto(user), tokens: this.issueTokens(user) };
  }

  async login(input: LoginDto): Promise<{ user: PublicUserDto; tokens: TokenPair }> {
    logger.debug({ email: input.email }, "auth.login: attempt");
    const user = await this.userRepository.findByEmailWithPassword(input.email);

    if (!user || !user.passwordHash) {
      logger.debug({ email: input.email }, "auth.login: rejected, no local password set");
      throw new UnauthorizedError("Invalid email or password");
    }

    const isValid = await comparePassword(input.password, user.passwordHash);
    if (!isValid) {
      logger.debug({ userId: user._id.toString() }, "auth.login: rejected, bad password");
      throw new UnauthorizedError("Invalid email or password");
    }

    logger.debug({ userId: user._id.toString() }, "auth.login: success");
    return { user: toPublicUserDto(user), tokens: this.issueTokens(user) };
  }

  async handleGoogleLogin(profile: GoogleProfile): Promise<{ user: PublicUserDto; tokens: TokenPair }> {
    let user = await this.userRepository.findByGoogleId(profile.googleId);

    if (!user) {
      const existingByEmail = await this.userRepository.findByEmail(profile.email);

      if (existingByEmail) {
        user = await this.userRepository.linkGoogleAccount(
          existingByEmail._id.toString(),
          profile.googleId,
        );
      } else {
        user = await this.userRepository.create({
          email: profile.email,
          name: profile.name,
          role: this.resolveRole(profile.email),
          authProviders: ["google"],
          googleId: profile.googleId,
          avatarUrl: profile.avatarUrl,
        });
      }
    }

    if (!user) {
      throw new UnauthorizedError("Unable to authenticate with Google");
    }

    return { user: toPublicUserDto(user), tokens: this.issueTokens(user) };
  }

  async refresh(refreshToken: string): Promise<TokenPair> {
    let payload;
    try {
      payload = verifyRefreshToken(refreshToken);
    } catch {
      throw new UnauthorizedError("Invalid or expired refresh token");
    }

    const user = await this.userRepository.findById(payload.sub);
    if (!user || user.refreshTokenVersion !== payload.tokenVersion) {
      throw new UnauthorizedError("Refresh token has been revoked");
    }

    return this.issueTokens(user);
  }

  async logout(userId: string): Promise<void> {
    logger.debug({ userId }, "auth.logout: revoking refresh tokens");
    await this.userRepository.incrementRefreshTokenVersion(userId);
  }

  /**
   * Lets an authenticated user (typically one who signed up via Google and
   * has no passwordHash yet) add local email/password login to their
   * existing account, without displacing their Google linkage.
   */
  async setPassword(userId: string, input: SetPasswordDto): Promise<PublicUserDto> {
    const passwordHash = await hashPassword(input.password);
    const user = await this.userRepository.setPassword(userId, passwordHash);
    if (!user) {
      throw new NotFoundError("User not found");
    }
    return toPublicUserDto(user);
  }

  /**
   * Always resolves successfully, whether or not the email exists, so the
   * endpoint can't be used to enumerate registered accounts. Silently does
   * nothing (but still returns normally) if there's no matching user.
   */
  async forgotPassword(input: ForgotPasswordDto): Promise<void> {
    const user = await this.userRepository.findByEmail(input.email);
    if (!user) {
      logger.debug({ email: input.email }, "auth.forgotPassword: no account, no-op");
      return;
    }

    await this.otpRepository.invalidateActiveForUser(user._id.toString(), "password-reset");

    const otp = generateOtp();
    const otpHash = await hashOtp(otp);
    const expiresAt = new Date(Date.now() + config.otp.expiryMinutes * 60 * 1000);
    await this.otpRepository.create(user._id.toString(), "password-reset", otpHash, expiresAt);

    await this.emailOtpSender.sendOtp(user.email, otp);
    logger.debug({ userId: user._id.toString() }, "auth.forgotPassword: OTP issued");
  }

  /**
   * Validates the OTP and exchanges it for a short-lived, single-purpose
   * reset token — the OTP itself is never accepted again, so it can't be
   * replayed if the reset-password step is delayed or retried.
   */
  async verifyOtp(input: VerifyOtpDto): Promise<{ resetToken: string }> {
    const user = await this.userRepository.findByEmail(input.email);
    if (!user) {
      throw new UnauthorizedError("Invalid or expired code");
    }

    const otpRecord = await this.otpRepository.findLatestActiveForUser(
      user._id.toString(),
      "password-reset",
    );
    if (!otpRecord) {
      throw new UnauthorizedError("Invalid or expired code");
    }

    const isValid = await compareOtp(input.otp, otpRecord.otpHash);
    if (!isValid) {
      logger.debug({ userId: user._id.toString() }, "auth.verifyOtp: bad code");
      throw new UnauthorizedError("Invalid or expired code");
    }

    await this.otpRepository.markConsumed(otpRecord._id.toString());
    logger.debug({ userId: user._id.toString() }, "auth.verifyOtp: verified");

    return { resetToken: signResetToken(user._id.toString()) };
  }

  async resetPassword(input: ResetPasswordDto): Promise<void> {
    let payload;
    try {
      payload = verifyResetToken(input.resetToken);
    } catch {
      throw new UnauthorizedError("Invalid or expired reset token");
    }

    const passwordHash = await hashPassword(input.newPassword);
    const user = await this.userRepository.setPassword(payload.sub, passwordHash);
    if (!user) {
      throw new NotFoundError("User not found");
    }

    // Revoke every outstanding refresh token — a password reset should log
    // the account out everywhere, not just on the device that reset it.
    await this.userRepository.incrementRefreshTokenVersion(payload.sub);
    logger.debug({ userId: payload.sub }, "auth.resetPassword: password reset, sessions revoked");
  }

  private resolveRole(email: string): UserRole {
    return config.adminEmails.includes(email.toLowerCase()) ? "admin" : "user";
  }

  private issueTokens(user: IUserDocument): TokenPair {
    return {
      accessToken: signAccessToken({ sub: user._id.toString(), role: user.role }),
      refreshToken: signRefreshToken({
        sub: user._id.toString(),
        tokenVersion: user.refreshTokenVersion,
      }),
    };
  }
}
