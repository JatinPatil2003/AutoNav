import { IPasswordResetOtpDocument } from "./passwordResetOtp.model";

export interface IPasswordResetOtpRepository {
  create(userId: string, otpHash: string, expiresAt: Date): Promise<IPasswordResetOtpDocument>;
  findLatestActiveForUser(userId: string): Promise<IPasswordResetOtpDocument | null>;
  markConsumed(id: string): Promise<void>;
  /** Invalidates any still-active OTPs for this user (e.g. before issuing a new one). */
  invalidateActiveForUser(userId: string): Promise<void>;
}
