import { Document, Model, Schema, Types, model } from "mongoose";

export type AuthProvider = "local" | "google";
export type UserRole = "user" | "admin";

export interface IUser {
  email: string;
  passwordHash?: string;
  authProviders: AuthProvider[];
  googleId?: string;
  name: string;
  avatarUrl?: string;
  role: UserRole;
  refreshTokenVersion: number;
  isEmailVerified: boolean;
}

export interface IUserDocument extends IUser, Document {
  _id: Types.ObjectId;
  createdAt: Date;
  updatedAt: Date;
}

const userSchema = new Schema<IUserDocument>(
  {
    email: { type: String, required: true, unique: true, lowercase: true, trim: true },
    passwordHash: { type: String, select: false },
    authProviders: {
      type: [String],
      enum: ["local", "google"],
      required: true,
      default: [],
    },
    googleId: { type: String, unique: true, sparse: true },
    name: { type: String, required: true, trim: true },
    avatarUrl: { type: String },
    role: { type: String, enum: ["user", "admin"], required: true, default: "user" },
    refreshTokenVersion: { type: Number, required: true, default: 0 },
    isEmailVerified: { type: Boolean, required: true, default: false },
  },
  { timestamps: true },
);

export const UserModel: Model<IUserDocument> = model<IUserDocument>("User", userSchema);
