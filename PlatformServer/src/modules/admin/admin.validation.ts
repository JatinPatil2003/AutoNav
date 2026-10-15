import { z } from "zod";

export const updateUserRoleSchema = z.object({
  role: z.enum(["user", "admin"]),
});

export const grantRobotAssignmentSchema = z.object({
  userId: z.string().min(1, "userId is required"),
  robotId: z.string().min(1, "robotId is required"),
  hmiAccess: z.boolean().optional(),
  platformAccess: z.boolean().optional(),
});

export const revokeRobotAssignmentSchema = z.object({
  reason: z.string().max(500).optional(),
});

export const updateRobotAssignmentAccessSchema = z
  .object({
    hmiAccess: z.boolean().optional(),
    platformAccess: z.boolean().optional(),
  })
  .refine((body) => body.hmiAccess !== undefined || body.platformAccess !== undefined, {
    message: "At least one of hmiAccess or platformAccess must be given",
  });

export type UpdateUserRoleDto = z.infer<typeof updateUserRoleSchema>;
export type GrantRobotAssignmentDto = z.infer<typeof grantRobotAssignmentSchema>;
export type RevokeRobotAssignmentDto = z.infer<typeof revokeRobotAssignmentSchema>;
export type UpdateRobotAssignmentAccessDto = z.infer<typeof updateRobotAssignmentAccessSchema>;
