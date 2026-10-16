import { z } from "zod";

/**
 * Mirrors PlatformServer/src/modules/auth/auth.validation.ts (same rules
 * as PlatformApp's copy). Kept in sync by hand for now — a shared
 * `packages/shared-types` workspace is the natural upgrade if
 * backend/frontend validation drifts.
 */
export const loginFormSchema = z.object({
  email: z.string().email("Enter a valid email"),
  password: z.string().min(1, "Password is required"),
});

export const signupFormSchema = z.object({
  name: z.string().min(1, "Name is required").max(100),
  email: z.string().email("Enter a valid email"),
  password: z.string().min(8, "Password must be at least 8 characters"),
});

export const forgotPasswordFormSchema = z.object({
  email: z.string().email("Enter a valid email"),
});

export const verifyOtpFormSchema = z.object({
  otp: z.string().length(6, "Enter the 6-digit code"),
});

export const resetPasswordFormSchema = z
  .object({
    newPassword: z.string().min(8, "Password must be at least 8 characters"),
    confirmPassword: z.string().min(1, "Please confirm your password"),
  })
  .refine((data) => data.newPassword === data.confirmPassword, {
    message: "Passwords don't match",
    path: ["confirmPassword"],
  });

export type LoginFormValues = z.infer<typeof loginFormSchema>;
export type SignupFormValues = z.infer<typeof signupFormSchema>;
export type ForgotPasswordFormValues = z.infer<typeof forgotPasswordFormSchema>;
export type VerifyOtpFormValues = z.infer<typeof verifyOtpFormSchema>;
export type ResetPasswordFormValues = z.infer<typeof resetPasswordFormSchema>;
