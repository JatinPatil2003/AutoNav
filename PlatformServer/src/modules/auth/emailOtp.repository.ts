import { EmailOtpModel, EmailOtpPurpose, IEmailOtpDocument } from "./emailOtp.model";
import { IEmailOtpRepository } from "./emailOtp.repository.interface";

export class MongooseEmailOtpRepository implements IEmailOtpRepository {
  async create(
    userId: string,
    purpose: EmailOtpPurpose,
    otpHash: string,
    expiresAt: Date,
  ): Promise<IEmailOtpDocument> {
    const otp = new EmailOtpModel({ user: userId, purpose, otpHash, expiresAt });
    return otp.save();
  }

  async findLatestActiveForUser(
    userId: string,
    purpose: EmailOtpPurpose,
  ): Promise<IEmailOtpDocument | null> {
    return EmailOtpModel.findOne({
      user: userId,
      purpose,
      consumed: false,
      expiresAt: { $gt: new Date() },
    })
      .sort({ createdAt: -1 })
      .exec();
  }

  async markConsumed(id: string): Promise<void> {
    await EmailOtpModel.findByIdAndUpdate(id, { $set: { consumed: true } }).exec();
  }

  async invalidateActiveForUser(userId: string, purpose: EmailOtpPurpose): Promise<void> {
    await EmailOtpModel.updateMany(
      { user: userId, purpose, consumed: false },
      { $set: { consumed: true } },
    ).exec();
  }
}
