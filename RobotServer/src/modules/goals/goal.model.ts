import { Document, Model, Schema, model } from "mongoose";

export interface IGoal {
  mapName: string;
  name: string;
  x: number;
  y: number;
  theta: number;
}

export interface IGoalDocument extends IGoal, Document {
  createdAt: Date;
  updatedAt: Date;
}

const goalSchema = new Schema<IGoalDocument>(
  {
    mapName: { type: String, required: true, index: true },
    name: { type: String, required: true },
    x: { type: Number, required: true },
    y: { type: Number, required: true },
    theta: { type: Number, required: true },
  },
  { timestamps: true },
);

// A given map can't have two goals with the same name — but "temp" (where
// in-progress mapping poses live before being promoted to a real map name)
// is reused across mapping sessions, so this is scoped per mapName.
goalSchema.index({ mapName: 1, name: 1 }, { unique: true });

export const GoalModel: Model<IGoalDocument> = model<IGoalDocument>("Goal", goalSchema);
