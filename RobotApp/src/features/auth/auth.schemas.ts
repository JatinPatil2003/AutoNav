import { z } from "zod";

/** Mirrors RobotServer/src/modules/robot-auth/robotAuth.validation.ts. */
export const identifyFormSchema = z.object({
  email: z.string().email("Enter a valid email"),
});

export const pinLoginFormSchema = z.object({
  pin: z.string().regex(/^\d{4}$/, "Enter your 4-digit PIN"),
});

export const otpFormSchema = z.object({
  otp: z.string().length(6, "Enter the 6-digit code"),
});

export const setPinFormSchema = z
  .object({
    pin: z.string().regex(/^\d{4}$/, "Enter a 4-digit PIN"),
    confirmPin: z.string().regex(/^\d{4}$/, "Confirm your 4-digit PIN"),
  })
  .refine((data) => data.pin === data.confirmPin, {
    message: "PINs don't match",
    path: ["confirmPin"],
  });

export type IdentifyFormValues = z.infer<typeof identifyFormSchema>;
export type PinLoginFormValues = z.infer<typeof pinLoginFormSchema>;
export type OtpFormValues = z.infer<typeof otpFormSchema>;
export type SetPinFormValues = z.infer<typeof setPinFormSchema>;
