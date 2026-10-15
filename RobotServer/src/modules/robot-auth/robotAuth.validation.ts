import { z } from "zod";

export const identifySchema = z.object({
  email: z.string().email(),
});

export const loginSchema = z.object({
  email: z.string().email(),
  pin: z.string().regex(/^\d{4}$/, "pin must be exactly 4 digits"),
});

export const refreshSchema = z.object({
  refreshToken: z.string().min(1).optional(),
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
