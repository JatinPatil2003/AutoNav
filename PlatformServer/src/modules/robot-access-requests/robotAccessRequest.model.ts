import { Document, Model, Schema, Types, model } from "mongoose";

export type RobotAccessRequestStatus = "pending" | "approved" | "rejected";

export interface IRobotAccessRequest {
  user: Types.ObjectId;
  robot: Types.ObjectId;
  status: RobotAccessRequestStatus;
  reviewedBy?: Types.ObjectId;
  reviewedAt?: Date;
  rejectionReason?: string;
}

export interface IRobotAccessRequestDocument extends IRobotAccessRequest, Document {
  _id: Types.ObjectId;
  createdAt: Date;
  updatedAt: Date;
}

const robotAccessRequestSchema = new Schema<IRobotAccessRequestDocument>(
  {
    user: { type: Schema.Types.ObjectId, ref: "User", required: true },
    robot: { type: Schema.Types.ObjectId, ref: "Robot", required: true },
    status: {
      type: String,
      enum: ["pending", "approved", "rejected"],
      required: true,
      default: "pending",
    },
    reviewedBy: { type: Schema.Types.ObjectId, ref: "User" },
    reviewedAt: { type: Date },
    rejectionReason: { type: String, trim: true },
  },
  { timestamps: true },
);

// Prevents a user from having more than one simultaneous pending request for
// the *same* robot; a user may still have concurrent pending requests for
// different robots now that User<->Robot is many-to-many.
robotAccessRequestSchema.index(
  { user: 1, robot: 1 },
  { unique: true, partialFilterExpression: { status: "pending" } },
);

export const RobotAccessRequestModel: Model<IRobotAccessRequestDocument> = model<IRobotAccessRequestDocument>(
  "RobotAccessRequest",
  robotAccessRequestSchema,
);
