import { z } from "zod";

export const startMappingSchema = z.object({
  backend: z.enum(["cartographer", "slam_toolbox"]).optional(),
});

export const waypointSchema = z.object({
  name: z.string().min(1),
});

export const mapNameSchema = z.object({
  mapName: z.string().min(1),
});

export const goalSchema = z.object({
  mapName: z.string().min(1),
  name: z.string().min(1),
  x: z.number(),
  y: z.number(),
  theta: z.number(),
});

export const startGoalSchema = z.object({
  mapName: z.string().min(1),
  name: z.string().min(1),
});
