import crypto from "crypto";
import bcrypt from "bcryptjs";

const OTP_SALT_ROUNDS = 10;

export function generateOtp(): string {
  return crypto.randomInt(0, 1_000_000).toString().padStart(6, "0");
}

export async function hashOtp(otp: string): Promise<string> {
  return bcrypt.hash(otp, OTP_SALT_ROUNDS);
}

export async function compareOtp(otp: string, otpHash: string): Promise<boolean> {
  return bcrypt.compare(otp, otpHash);
}
