import { EmailOtpPurpose, IEmailOtpDocument } from "./emailOtp.model";

export interface IEmailOtpRepository {
  create(
    userId: string,
    purpose: EmailOtpPurpose,
    otpHash: string,
    expiresAt: Date,
  ): Promise<IEmailOtpDocument>;
  findLatestActiveForUser(userId: string, purpose: EmailOtpPurpose): Promise<IEmailOtpDocument | null>;
  markConsumed(id: string): Promise<void>;
  /** Invalidates any still-active OTPs for this user+purpose (e.g. before issuing a new one). */
  invalidateActiveForUser(userId: string, purpose: EmailOtpPurpose): Promise<void>;
}
