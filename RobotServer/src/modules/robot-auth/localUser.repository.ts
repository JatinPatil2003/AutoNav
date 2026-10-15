import { RosterEntryDto } from "../../clients/platformClient.interface";
import { ILocalUserDocument, LocalUserModel } from "./localUser.model";
import { ILocalUserRepository } from "./localUser.repository.interface";

export class MongooseLocalUserRepository implements ILocalUserRepository {
  async findByEmail(email: string, robotId: string): Promise<ILocalUserDocument | null> {
    return LocalUserModel.findOne({ email: email.toLowerCase(), robotId }).exec();
  }

  async listActive(robotId: string): Promise<ILocalUserDocument[]> {
    return LocalUserModel.find({ status: "active", robotId }).sort({ name: 1 }).exec();
  }

  async replaceAll(entries: RosterEntryDto[], robotId: string): Promise<void> {
    const seenIds = entries.map((e) => e.userId);

    await Promise.all(
      entries.map((entry) =>
        LocalUserModel.updateOne(
          { platformUserId: entry.userId },
          {
            $set: {
              robotId,
              email: entry.email.toLowerCase(),
              name: entry.name,
              avatarUrl: entry.avatarUrl,
              pinHash: entry.pinHash,
              pinUpdatedCount: entry.pinUpdatedCount,
              status: "active",
              syncedAt: new Date(),
            },
          },
          { upsert: true },
        ).exec(),
      ),
    );

    // Anyone cached locally for this robot but no longer in the active
    // roster (revoked, reassigned, etc.) is marked revoked rather than
    // deleted, preserving the lockout/attempt history if they're ever
    // re-granted access.
    await LocalUserModel.updateMany(
      { robotId, platformUserId: { $nin: seenIds }, status: "active" },
      { $set: { status: "revoked", syncedAt: new Date() } },
    ).exec();
  }

  async applyEntry(entry: RosterEntryDto, robotId: string): Promise<ILocalUserDocument> {
    const updated = await LocalUserModel.findOneAndUpdate(
      { platformUserId: entry.userId },
      {
        $set: {
          robotId,
          email: entry.email.toLowerCase(),
          name: entry.name,
          avatarUrl: entry.avatarUrl,
          pinHash: entry.pinHash,
          pinUpdatedCount: entry.pinUpdatedCount,
          status: "active",
          syncedAt: new Date(),
        },
      },
      { upsert: true, new: true },
    ).exec();
    return updated!;
  }

  async recordFailedAttempt(
    id: string,
    maxAttempts: number,
    lockoutMinutes: number,
  ): Promise<ILocalUserDocument | null> {
    const user = await LocalUserModel.findById(id).exec();
    if (!user) return null;

    user.failedAttempts += 1;
    if (user.failedAttempts >= maxAttempts) {
      user.lockedUntil = new Date(Date.now() + lockoutMinutes * 60 * 1000);
    }
    return user.save();
  }

  async resetAttempts(id: string): Promise<void> {
    await LocalUserModel.findByIdAndUpdate(id, {
      $set: { failedAttempts: 0, lockedUntil: undefined },
    }).exec();
  }
}
