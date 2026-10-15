import { z } from "zod";

export const waypointSchema = z.object({
  name: z.string().min(1),
});

export const saveMapSchema = z.object({
  mapName: z.string().min(1),
});

export type WaypointDto = z.infer<typeof waypointSchema>;
export type SaveMapDto = z.infer<typeof saveMapSchema>;
