import { IUserDocument, UserRole } from "./user.model";

export interface CreateUserInput {
  email: string;
  name: string;
  role: "user" | "admin";
  authProviders: ("local" | "google")[];
  passwordHash?: string;
  googleId?: string;
  avatarUrl?: string;
}

export interface IUserRepository {
  findById(id: string): Promise<IUserDocument | null>;
  findByIdPopulated(id: string): Promise<IUserDocument | null>;
  findByEmail(email: string): Promise<IUserDocument | null>;
  findByEmailWithPassword(email: string): Promise<IUserDocument | null>;
  findByGoogleId(googleId: string): Promise<IUserDocument | null>;
  create(input: CreateUserInput): Promise<IUserDocument>;
  linkGoogleAccount(userId: string, googleId: string): Promise<IUserDocument | null>;
  incrementRefreshTokenVersion(userId: string): Promise<void>;
  assignRobot(userId: string, robotId: string): Promise<IUserDocument | null>;
  setPassword(userId: string, passwordHash: string): Promise<IUserDocument | null>;
  listAllPopulated(): Promise<IUserDocument[]>;
  updateRole(userId: string, role: UserRole): Promise<IUserDocument | null>;
}
