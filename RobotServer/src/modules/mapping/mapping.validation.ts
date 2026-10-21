import { z } from "zod";
import { DEFAULT_SLAM_BACKEND } from "./mapping.types";

export const startMappingSchema = z.object({
  backend: z.enum(["cartographer", "slam_toolbox"]).default(DEFAULT_SLAM_BACKEND),
});

export const waypointSchema = z.object({
  name: z.string().min(1),
});

export const saveMapSchema = z.object({
  mapName: z.string().min(1),
});

export type StartMappingDto = z.infer<typeof startMappingSchema>;
export type WaypointDto = z.infer<typeof waypointSchema>;
export type SaveMapDto = z.infer<typeof saveMapSchema>;
