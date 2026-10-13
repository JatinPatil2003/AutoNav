import { Types } from "mongoose";
import { IPasswordResetOtpDocument } from "../../../src/modules/auth/passwordResetOtp.model";
import { IPasswordResetOtpRepository } from "../../../src/modules/auth/passwordResetOtp.repository.interface";

export class FakePasswordResetOtpRepository implements IPasswordResetOtpRepository {
  private otps = new Map<string, IPasswordResetOtpDocument>();

  async create(userId: string, otpHash: string, expiresAt: Date): Promise<IPasswordResetOtpDocument> {
    const id = new Types.ObjectId();
    const otp = {
      _id: id,
      user: new Types.ObjectId(userId),
      otpHash,
      expiresAt,
      consumed: false,
      createdAt: new Date(),
    } as unknown as IPasswordResetOtpDocument;

    this.otps.set(id.toString(), otp);
    return otp;
  }

  async findLatestActiveForUser(userId: string): Promise<IPasswordResetOtpDocument | null> {
    const now = new Date();
    const active = [...this.otps.values()]
      .filter((o) => o.user.toString() === userId && !o.consumed && o.expiresAt > now)
      .sort((a, b) => b.createdAt.getTime() - a.createdAt.getTime());
    return active[0] ?? null;
  }

  async markConsumed(id: string): Promise<void> {
    const otp = this.otps.get(id);
    if (otp) otp.consumed = true;
  }

  async invalidateActiveForUser(userId: string): Promise<void> {
    for (const otp of this.otps.values()) {
      if (otp.user.toString() === userId && !otp.consumed) {
        otp.consumed = true;
      }
    }
  }
}
