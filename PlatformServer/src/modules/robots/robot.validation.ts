import { z } from "zod";

export const createRobotSchema = z.object({
  name: z.string().min(2).max(64),
  vpnHostname: z.string().min(2).max(128),
  displayName: z.string().max(128).optional(),
});

export type CreateRobotDto = z.infer<typeof createRobotSchema>;
