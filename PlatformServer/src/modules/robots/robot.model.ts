import { Document, Model, Schema, Types, model } from "mongoose";

export type RobotStatus = "provisioned" | "active" | "retired";

export interface IRobot {
  robotId: string;
  name: string;
  vpnHostname: string;
  displayName?: string;
  status: RobotStatus;
  lastSeenAt?: Date;
}

export interface IRobotDocument extends IRobot, Document {
  _id: Types.ObjectId;
  createdAt: Date;
  updatedAt: Date;
}

const robotSchema = new Schema<IRobotDocument>(
  {
    robotId: { type: String, required: true, unique: true, trim: true, lowercase: true },
    name: { type: String, required: true, unique: true, trim: true },
    vpnHostname: { type: String, required: true, unique: true, trim: true },
    displayName: { type: String, trim: true },
    status: {
      type: String,
      enum: ["provisioned", "active", "retired"],
      required: true,
      default: "provisioned",
    },
    lastSeenAt: { type: Date },
  },
  { timestamps: true },
);

export const RobotModel: Model<IRobotDocument> = model<IRobotDocument>("Robot", robotSchema);
