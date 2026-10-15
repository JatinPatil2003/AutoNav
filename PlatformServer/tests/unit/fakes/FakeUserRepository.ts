import { Types } from "mongoose";
import { IUserDocument } from "../../../src/modules/users/user.model";
import { CreateUserInput, IUserRepository } from "../../../src/modules/users/user.repository.interface";

/**
 * In-memory stand-in for IUserRepository. Exists to prove the service layer
 * only depends on the repository interface, not Mongoose — no real database
 * is started for these tests.
 */
export class FakeUserRepository implements IUserRepository {
  private users = new Map<string, IUserDocument>();

  async findById(id: string): Promise<IUserDocument | null> {
    return this.users.get(id) ?? null;
  }

  async findByIdPopulated(id: string): Promise<IUserDocument | null> {
    return this.findById(id);
  }

  async findByEmail(email: string): Promise<IUserDocument | null> {
    return (
      [...this.users.values()].find((u) => u.email === email.toLowerCase()) ?? null
    );
  }

  async findByEmailWithPassword(email: string): Promise<IUserDocument | null> {
    return this.findByEmail(email);
  }

  async findByGoogleId(googleId: string): Promise<IUserDocument | null> {
    return [...this.users.values()].find((u) => u.googleId === googleId) ?? null;
  }

  async create(input: CreateUserInput): Promise<IUserDocument> {
    const id = new Types.ObjectId();
    const user = {
      _id: id,
      email: input.email.toLowerCase(),
      name: input.name,
      role: input.role,
      authProviders: input.authProviders,
      passwordHash: input.passwordHash,
      googleId: input.googleId,
      avatarUrl: input.avatarUrl,
      refreshTokenVersion: 0,
      isEmailVerified: false,
      createdAt: new Date(),
      updatedAt: new Date(),
    } as unknown as IUserDocument;

    this.users.set(id.toString(), user);
    return user;
  }

  async linkGoogleAccount(userId: string, googleId: string): Promise<IUserDocument | null> {
    const user = this.users.get(userId);
    if (!user) return null;
    user.googleId = googleId;
    if (!user.authProviders.includes("google")) user.authProviders.push("google");
    return user;
  }

  async incrementRefreshTokenVersion(userId: string): Promise<void> {
    const user = this.users.get(userId);
    if (user) user.refreshTokenVersion += 1;
  }

  async setPassword(userId: string, passwordHash: string): Promise<IUserDocument | null> {
    const user = this.users.get(userId);
    if (!user) return null;
    user.passwordHash = passwordHash;
    if (!user.authProviders.includes("local")) user.authProviders.push("local");
    return user;
  }

  async listAllPopulated(): Promise<IUserDocument[]> {
    return [...this.users.values()];
  }

  async updateRole(userId: string, role: "user" | "admin"): Promise<IUserDocument | null> {
    const user = this.users.get(userId);
    if (!user) return null;
    user.role = role;
    return user;
  }

  async deleteById(userId: string): Promise<void> {
    this.users.delete(userId);
  }
}
