import { Document, Model, Schema, model } from "mongoose";

export type LocalUserStatus = "active" | "revoked";

export interface ILocalUser {
  /** This robot's own device id at the time this row was last synced — see ros/deviceIdentity.ts. */
  robotId: string;
  platformUserId: string;
  email: string;
  name: string;
  avatarUrl?: string;
  pinHash: string | null;
  pinUpdatedCount: number;
  status: LocalUserStatus;
  failedAttempts: number;
  lockedUntil?: Date;
  syncedAt: Date;
}

export interface ILocalUserDocument extends ILocalUser, Document {
  createdAt: Date;
  updatedAt: Date;
}

const localUserSchema = new Schema<ILocalUserDocument>(
  {
    robotId: { type: String, required: true, index: true },
    platformUserId: { type: String, required: true, unique: true },
    email: { type: String, required: true, index: true, lowercase: true, trim: true },
    name: { type: String, required: true },
    avatarUrl: { type: String },
    pinHash: { type: String, default: null },
    pinUpdatedCount: { type: Number, required: true, default: 0 },
    status: { type: String, enum: ["active", "revoked"], required: true, default: "active" },
    failedAttempts: { type: Number, required: true, default: 0 },
    lockedUntil: { type: Date },
    syncedAt: { type: Date, required: true, default: () => new Date() },
  },
  { timestamps: true },
);

// Persisted (not in-memory-only) so a reboot during a VPN blip doesn't
// strand PIN login — see rosterSync.service.ts and robotAuth.service.ts.
export const LocalUserModel: Model<ILocalUserDocument> = model<ILocalUserDocument>(
  "LocalUser",
  localUserSchema,
);
