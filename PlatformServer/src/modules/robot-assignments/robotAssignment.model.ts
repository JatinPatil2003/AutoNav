import { Document, Model, Schema, Types, model } from "mongoose";

export type RobotAssignmentStatus = "active" | "revoked";

export interface IRobotAssignment {
  user: Types.ObjectId;
  robot: Types.ObjectId;
  status: RobotAssignmentStatus;
  /** Can log into this robot's own RobotApp kiosk (PIN login). */
  hmiAccess: boolean;
  /** Shows up in this user's "my robots" list in PlatformApp. */
  platformAccess: boolean;
  pinHash?: string;
  pinSetAt?: Date;
  pinUpdatedCount: number;
  grantedBy: Types.ObjectId;
  grantedAt: Date;
  revokedBy?: Types.ObjectId;
  revokedAt?: Date;
  revokedReason?: string;
}

export interface IRobotAssignmentDocument extends IRobotAssignment, Document {
  _id: Types.ObjectId;
  createdAt: Date;
  updatedAt: Date;
}

const robotAssignmentSchema = new Schema<IRobotAssignmentDocument>(
  {
    user: { type: Schema.Types.ObjectId, ref: "User", required: true },
    robot: { type: Schema.Types.ObjectId, ref: "Robot", required: true },
    status: { type: String, enum: ["active", "revoked"], required: true, default: "active" },
    hmiAccess: { type: Boolean, required: true, default: true },
    platformAccess: { type: Boolean, required: true, default: true },
    pinHash: { type: String, select: false },
    pinSetAt: { type: Date },
    pinUpdatedCount: { type: Number, required: true, default: 0 },
    grantedBy: { type: Schema.Types.ObjectId, ref: "User", required: true },
    grantedAt: { type: Date, required: true, default: () => new Date() },
    revokedBy: { type: Schema.Types.ObjectId, ref: "User" },
    revokedAt: { type: Date },
    revokedReason: { type: String, trim: true },
  },
  { timestamps: true },
);

// One assignment doc per (user, robot) pair for all time — re-approval after
// a revoke reactivates the same doc rather than inserting a duplicate.
robotAssignmentSchema.index({ user: 1, robot: 1 }, { unique: true });
robotAssignmentSchema.index({ robot: 1, status: 1 });

export const RobotAssignmentModel: Model<IRobotAssignmentDocument> = model<IRobotAssignmentDocument>(
  "RobotAssignment",
  robotAssignmentSchema,
);
