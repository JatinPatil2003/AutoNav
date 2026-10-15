import { z } from "zod";

export const createRequestSchema = z.object({
  robotId: z.string().min(1, "robotId is required"),
});

export const approveRequestSchema = z.object({
  robotId: z.string().min(1).optional(),
});

export const rejectRequestSchema = z.object({
  reason: z.string().max(500).optional(),
});

export type ApproveRequestDto = z.infer<typeof approveRequestSchema>;
export type RejectRequestDto = z.infer<typeof rejectRequestSchema>;
