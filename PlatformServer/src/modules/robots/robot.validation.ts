import { z } from "zod";

export const createRobotSchema = z.object({
  robotId: z
    .string()
    .min(3)
    .max(32)
    .regex(/^[A-Za-z0-9-]+$/, "robotId may only contain letters, digits, and hyphens"),
  name: z.string().min(2).max(64),
  vpnHostname: z.string().min(2).max(128),
  displayName: z.string().max(128).optional(),
});

export type CreateRobotDto = z.infer<typeof createRobotSchema>;
