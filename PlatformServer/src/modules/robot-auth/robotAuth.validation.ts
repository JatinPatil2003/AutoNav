import { z } from "zod";

export const checkUserExistsSchema = z.object({
  email: z.string().email(),
});

export const requestPinSetupOtpSchema = z.object({
  email: z.string().email(),
});

export const verifyPinSetupOtpSchema = z.object({
  email: z.string().email(),
  otp: z.string().length(6),
});

export const confirmPinSetupSchema = z.object({
  pinSetupToken: z.string().min(1),
  pin: z.string().regex(/^\d{4}$/, "pin must be exactly 4 digits"),
});

export type CheckUserExistsDto = z.infer<typeof checkUserExistsSchema>;
export type RequestPinSetupOtpDto = z.infer<typeof requestPinSetupOtpSchema>;
export type VerifyPinSetupOtpDto = z.infer<typeof verifyPinSetupOtpSchema>;
export type ConfirmPinSetupDto = z.infer<typeof confirmPinSetupSchema>;
