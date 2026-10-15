import { Document, Model, Schema, Types, model } from "mongoose";

export type RobotStatus = "provisioned" | "active" | "retired";

export interface IEncryptedSecret {
  ciphertext: string;
  iv: string;
  authTag: string;
}

export interface IRobot {
  robotId: string;
  name: string;
  vpnHostname: string;
  displayName?: string;
  status: RobotStatus;
  robotSecretEncrypted: IEncryptedSecret;
  /** SHA-256 of the plaintext secret — lets requireRobotAuth identify the robot from the secret alone. */
  robotSecretHash: string;
  robotAuthTokenVersion: number;
  lastSeenAt?: Date;
}

export interface IRobotDocument extends IRobot, Document {
  _id: Types.ObjectId;
  createdAt: Date;
  updatedAt: Date;
}

const encryptedSecretSchema = new Schema<IEncryptedSecret>(
  {
    ciphertext: { type: String, required: true },
    iv: { type: String, required: true },
    authTag: { type: String, required: true },
  },
  { _id: false },
);

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
    robotSecretEncrypted: { type: encryptedSecretSchema, required: true },
    robotSecretHash: { type: String, required: true, unique: true },
    robotAuthTokenVersion: { type: Number, required: true, default: 0 },
    lastSeenAt: { type: Date },
  },
  { timestamps: true },
);

export const RobotModel: Model<IRobotDocument> = model<IRobotDocument>("Robot", robotSchema);
