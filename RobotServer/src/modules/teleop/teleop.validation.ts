import { z } from "zod";

export const velocitySchema = z.object({
  linear: z.number(),
  angular: z.number(),
});
export type VelocityDto = z.infer<typeof velocitySchema>;

export const emergencySchema = z.object({
  status: z.boolean(),
});
export type EmergencyDto = z.infer<typeof emergencySchema>;
