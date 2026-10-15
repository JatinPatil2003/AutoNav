import { Types } from "mongoose";
import { EmailOtpPurpose, IEmailOtpDocument } from "../../../src/modules/auth/emailOtp.model";
import { IEmailOtpRepository } from "../../../src/modules/auth/emailOtp.repository.interface";

export class FakeEmailOtpRepository implements IEmailOtpRepository {
  private otps = new Map<string, IEmailOtpDocument>();

  async create(
    userId: string,
    purpose: EmailOtpPurpose,
    otpHash: string,
    expiresAt: Date,
  ): Promise<IEmailOtpDocument> {
    const id = new Types.ObjectId();
    const otp = {
      _id: id,
      user: new Types.ObjectId(userId),
      purpose,
      otpHash,
      expiresAt,
      consumed: false,
      createdAt: new Date(),
    } as unknown as IEmailOtpDocument;

    this.otps.set(id.toString(), otp);
    return otp;
  }

  async findLatestActiveForUser(
    userId: string,
    purpose: EmailOtpPurpose,
  ): Promise<IEmailOtpDocument | null> {
    const now = new Date();
    const active = [...this.otps.values()]
      .filter((o) => o.user.toString() === userId && o.purpose === purpose && !o.consumed && o.expiresAt > now)
      .sort((a, b) => b.createdAt.getTime() - a.createdAt.getTime());
    return active[0] ?? null;
  }

  async markConsumed(id: string): Promise<void> {
    const otp = this.otps.get(id);
    if (otp) otp.consumed = true;
  }

  async invalidateActiveForUser(userId: string, purpose: EmailOtpPurpose): Promise<void> {
    for (const otp of this.otps.values()) {
      if (otp.user.toString() === userId && otp.purpose === purpose && !otp.consumed) {
        otp.consumed = true;
      }
    }
  }
}
