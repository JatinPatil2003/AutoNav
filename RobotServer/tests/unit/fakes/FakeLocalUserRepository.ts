import { RosterEntryDto } from "../../../src/clients/platformClient.interface";
import { ILocalUserDocument } from "../../../src/modules/robot-auth/localUser.model";
import { ILocalUserRepository } from "../../../src/modules/robot-auth/localUser.repository.interface";

let nextId = 1;

export class FakeLocalUserRepository implements ILocalUserRepository {
  private users = new Map<string, ILocalUserDocument>();

  async findByEmail(email: string): Promise<ILocalUserDocument | null> {
    return [...this.users.values()].find((u) => u.email === email.toLowerCase()) ?? null;
  }

  async listActive(): Promise<ILocalUserDocument[]> {
    return [...this.users.values()]
      .filter((u) => u.status === "active")
      .sort((a, b) => a.name.localeCompare(b.name));
  }

  async replaceAll(entries: RosterEntryDto[]): Promise<void> {
    const seenIds = new Set(entries.map((e) => e.userId));

    for (const entry of entries) {
      const existing = [...this.users.values()].find((u) => u.platformUserId === entry.userId);
      if (existing) {
        existing.email = entry.email.toLowerCase();
        existing.name = entry.name;
        existing.pinHash = entry.pinHash;
        existing.pinUpdatedCount = entry.pinUpdatedCount;
        existing.status = "active";
      } else {
        this.create(entry);
      }
    }

    for (const user of this.users.values()) {
      if (!seenIds.has(user.platformUserId) && user.status === "active") {
        user.status = "revoked";
      }
    }
  }

  async applyEntry(entry: RosterEntryDto): Promise<ILocalUserDocument> {
    const existing = [...this.users.values()].find((u) => u.platformUserId === entry.userId);
    if (existing) {
      existing.email = entry.email.toLowerCase();
      existing.name = entry.name;
      existing.pinHash = entry.pinHash;
      existing.pinUpdatedCount = entry.pinUpdatedCount;
      existing.status = "active";
      return existing;
    }
    return this.create(entry);
  }

  async recordFailedAttempt(
    id: string,
    maxAttempts: number,
    lockoutMinutes: number,
  ): Promise<ILocalUserDocument | null> {
    const user = this.users.get(id);
    if (!user) return null;
    user.failedAttempts += 1;
    if (user.failedAttempts >= maxAttempts) {
      user.lockedUntil = new Date(Date.now() + lockoutMinutes * 60 * 1000);
    }
    return user;
  }

  async resetAttempts(id: string): Promise<void> {
    const user = this.users.get(id);
    if (user) {
      user.failedAttempts = 0;
      user.lockedUntil = undefined;
    }
  }

  private create(entry: RosterEntryDto): ILocalUserDocument {
    const id = String(nextId++);
    const user = {
      _id: id,
      platformUserId: entry.userId,
      email: entry.email.toLowerCase(),
      name: entry.name,
      pinHash: entry.pinHash,
      pinUpdatedCount: entry.pinUpdatedCount,
      status: "active",
      failedAttempts: 0,
      syncedAt: new Date(),
      createdAt: new Date(),
      updatedAt: new Date(),
    } as unknown as ILocalUserDocument;

    this.users.set(id, user);
    return user;
  }
}
