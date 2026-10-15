import { IUserDocument, UserModel } from "./user.model";
import { CreateUserInput, IUserRepository } from "./user.repository.interface";

export class MongooseUserRepository implements IUserRepository {
  async findById(id: string): Promise<IUserDocument | null> {
    return UserModel.findById(id).exec();
  }

  async findByIdPopulated(id: string): Promise<IUserDocument | null> {
    return UserModel.findById(id).exec();
  }

  async findByEmail(email: string): Promise<IUserDocument | null> {
    return UserModel.findOne({ email: email.toLowerCase() }).exec();
  }

  async findByEmailWithPassword(email: string): Promise<IUserDocument | null> {
    return UserModel.findOne({ email: email.toLowerCase() }).select("+passwordHash").exec();
  }

  async findByGoogleId(googleId: string): Promise<IUserDocument | null> {
    return UserModel.findOne({ googleId }).exec();
  }

  async create(input: CreateUserInput): Promise<IUserDocument> {
    const user = new UserModel({
      email: input.email.toLowerCase(),
      name: input.name,
      role: input.role,
      authProviders: input.authProviders,
      passwordHash: input.passwordHash,
      googleId: input.googleId,
      avatarUrl: input.avatarUrl,
    });
    return user.save();
  }

  async linkGoogleAccount(userId: string, googleId: string): Promise<IUserDocument | null> {
    return UserModel.findByIdAndUpdate(
      userId,
      { $set: { googleId }, $addToSet: { authProviders: "google" } },
      { new: true },
    ).exec();
  }

  async incrementRefreshTokenVersion(userId: string): Promise<void> {
    await UserModel.findByIdAndUpdate(userId, { $inc: { refreshTokenVersion: 1 } }).exec();
  }

  async setPassword(userId: string, passwordHash: string): Promise<IUserDocument | null> {
    return UserModel.findByIdAndUpdate(
      userId,
      { $set: { passwordHash }, $addToSet: { authProviders: "local" } },
      { new: true },
    ).exec();
  }

  async listAllPopulated(): Promise<IUserDocument[]> {
    return UserModel.find().sort({ createdAt: -1 }).exec();
  }

  async updateRole(userId: string, role: "user" | "admin"): Promise<IUserDocument | null> {
    return UserModel.findByIdAndUpdate(userId, { $set: { role } }, { new: true }).exec();
  }

  async deleteById(userId: string): Promise<void> {
    await UserModel.deleteOne({ _id: userId }).exec();
  }
}
