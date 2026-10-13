import { PasswordResetOtpModel, IPasswordResetOtpDocument } from "./passwordResetOtp.model";
import { IPasswordResetOtpRepository } from "./passwordResetOtp.repository.interface";

export class MongoosePasswordResetOtpRepository implements IPasswordResetOtpRepository {
  async create(userId: string, otpHash: string, expiresAt: Date): Promise<IPasswordResetOtpDocument> {
    const otp = new PasswordResetOtpModel({ user: userId, otpHash, expiresAt });
    return otp.save();
  }

  async findLatestActiveForUser(userId: string): Promise<IPasswordResetOtpDocument | null> {
    return PasswordResetOtpModel.findOne({
      user: userId,
      consumed: false,
      expiresAt: { $gt: new Date() },
    })
      .sort({ createdAt: -1 })
      .exec();
  }

  async markConsumed(id: string): Promise<void> {
    await PasswordResetOtpModel.findByIdAndUpdate(id, { $set: { consumed: true } }).exec();
  }

  async invalidateActiveForUser(userId: string): Promise<void> {
    await PasswordResetOtpModel.updateMany(
      { user: userId, consumed: false },
      { $set: { consumed: true } },
    ).exec();
  }
}
