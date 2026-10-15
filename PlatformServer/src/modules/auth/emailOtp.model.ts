import { Document, Model, Schema, Types, model } from "mongoose";

export type EmailOtpPurpose = "password-reset" | "robot-pin-setup";

export interface IEmailOtp {
  user: Types.ObjectId;
  purpose: EmailOtpPurpose;
  otpHash: string;
  expiresAt: Date;
  consumed: boolean;
}

export interface IEmailOtpDocument extends IEmailOtp, Document {
  _id: Types.ObjectId;
  createdAt: Date;
}

const emailOtpSchema = new Schema<IEmailOtpDocument>(
  {
    user: { type: Schema.Types.ObjectId, ref: "User", required: true, index: true },
    purpose: { type: String, enum: ["password-reset", "robot-pin-setup"], required: true },
    otpHash: { type: String, required: true },
    // TTL index: Mongo automatically deletes the document once expiresAt passes,
    // so stale/used OTPs don't need manual cleanup.
    expiresAt: { type: Date, required: true, expires: 0 },
    consumed: { type: Boolean, required: true, default: false },
  },
  { timestamps: { createdAt: true, updatedAt: false } },
);

emailOtpSchema.index({ user: 1, purpose: 1 });

export const EmailOtpModel: Model<IEmailOtpDocument> = model<IEmailOtpDocument>(
  "EmailOtp",
  emailOtpSchema,
);
