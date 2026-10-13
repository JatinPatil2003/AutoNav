import { Document, Model, Schema, Types, model } from "mongoose";

export type RobotStatus = "unassigned" | "assigned" | "retired";

export interface IRobot {
  name: string;
  vpnHostname: string;
  displayName?: string;
  status: RobotStatus;
  assignedTo?: Types.ObjectId;
}

export interface IRobotDocument extends IRobot, Document {
  _id: Types.ObjectId;
  createdAt: Date;
  updatedAt: Date;
}

const robotSchema = new Schema<IRobotDocument>(
  {
    name: { type: String, required: true, unique: true, trim: true },
    vpnHostname: { type: String, required: true, unique: true, trim: true },
    displayName: { type: String, trim: true },
    status: {
      type: String,
      enum: ["unassigned", "assigned", "retired"],
      required: true,
      default: "unassigned",
    },
    assignedTo: { type: Schema.Types.ObjectId, ref: "User", sparse: true },
  },
  { timestamps: true },
);

export const RobotModel: Model<IRobotDocument> = model<IRobotDocument>("Robot", robotSchema);
