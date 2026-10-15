import { z } from "zod";

export const mapNameSchema = z.object({
  mapName: z.string().min(1),
});

export const poseSchema = z.object({
  x: z.number(),
  y: z.number(),
  theta: z.number(),
});

export const goalSchema = poseSchema.extend({
  mapName: z.string().min(1),
  name: z.string().min(1),
});

export const goalLookupSchema = z.object({
  mapName: z.string().min(1),
  name: z.string().min(1),
});

export type MapNameDto = z.infer<typeof mapNameSchema>;
export type PoseDto = z.infer<typeof poseSchema>;
export type GoalDto = z.infer<typeof goalSchema>;
export type GoalLookupDto = z.infer<typeof goalLookupSchema>;
