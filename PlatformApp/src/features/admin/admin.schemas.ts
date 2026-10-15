import { z } from "zod";

/**
 * Mirrors PlatformServer/src/modules/robots/robot.validation.ts.
 */
export const createRobotFormSchema = z.object({
  robotId: z
    .string()
    .min(3, "At least 3 characters")
    .max(32)
    .regex(/^[A-Za-z0-9-]+$/, "Letters, digits, and hyphens only"),
  name: z.string().min(2, "At least 2 characters").max(64),
  vpnHostname: z.string().min(2, "At least 2 characters").max(128),
  displayName: z.string().max(128).optional().or(z.literal("")),
});

export type CreateRobotFormValues = z.infer<typeof createRobotFormSchema>;
