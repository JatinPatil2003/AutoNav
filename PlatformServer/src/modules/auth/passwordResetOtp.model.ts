import { Document, Model, Schema, Types, model } from "mongoose";

export interface IPasswordResetOtp {
  user: Types.ObjectId;
  otpHash: string;
  expiresAt: Date;
  consumed: boolean;
}

export interface IPasswordResetOtpDocument extends IPasswordResetOtp, Document {
  _id: Types.ObjectId;
  createdAt: Date;
}

const passwordResetOtpSchema = new Schema<IPasswordResetOtpDocument>(
  {
    user: { type: Schema.Types.ObjectId, ref: "User", required: true, index: true },
    otpHash: { type: String, required: true },
    // TTL index: Mongo automatically deletes the document once expiresAt passes,
    // so stale/used OTPs don't need manual cleanup.
    expiresAt: { type: Date, required: true, expires: 0 },
    consumed: { type: Boolean, required: true, default: false },
  },
  { timestamps: { createdAt: true, updatedAt: false } },
);

export const PasswordResetOtpModel: Model<IPasswordResetOtpDocument> = model<IPasswordResetOtpDocument>(
  "PasswordResetOtp",
  passwordResetOtpSchema,
);
