import "dotenv/config";
import { z } from "zod";

const envSchema = z.object({
  NODE_ENV: z.enum(["development", "test", "production"]).default("development"),
  PORT: z.coerce.number().int().positive().default(4000),

  MONGO_URI: z.string().min(1, "MONGO_URI is required"),

  JWT_ACCESS_SECRET: z.string().min(16, "JWT_ACCESS_SECRET must be at least 16 characters"),
  JWT_REFRESH_SECRET: z.string().min(16, "JWT_REFRESH_SECRET must be at least 16 characters"),
  ACCESS_TOKEN_TTL: z.string().default("15m"),
  REFRESH_TOKEN_TTL: z.string().default("30d"),

  ADMIN_EMAILS: z.string().default(""),

  FRONTEND_URL: z.string().url(),
  COOKIE_DOMAIN: z.string().min(1),

  GOOGLE_CLIENT_ID: z.string().min(1),
  GOOGLE_CLIENT_SECRET: z.string().min(1),
  GOOGLE_CALLBACK_URL: z.string().url(),

  EMAIL_USER: z.string().email(),
  EMAIL_APP_PASSWORD: z.string().min(1),

  OTP_EXPIRY_MINUTES: z.coerce.number().int().positive().default(10),
  RESET_TOKEN_SECRET: z.string().min(16, "RESET_TOKEN_SECRET must be at least 16 characters"),
  RESET_TOKEN_TTL: z.string().default("10m"),

  PIN_SETUP_TOKEN_SECRET: z.string().min(16, "PIN_SETUP_TOKEN_SECRET must be at least 16 characters"),
  PIN_SETUP_TOKEN_TTL: z.string().default("10m"),
  ROBOT_CALL_SCHEME: z.enum(["http", "https"]).default("https"),
  ROBOT_CALL_TIMEOUT_MS: z.coerce.number().int().positive().default(5000),
});

function loadConfig() {
  const parsed = envSchema.safeParse(process.env);

  if (!parsed.success) {
    const issues = parsed.error.issues
      .map((issue) => `  - ${issue.path.join(".")}: ${issue.message}`)
      .join("\n");
    throw new Error(`Invalid environment configuration:\n${issues}`);
  }

  const env = parsed.data;

  return {
    nodeEnv: env.NODE_ENV,
    isProduction: env.NODE_ENV === "production",
    isDevelopment: env.NODE_ENV === "development",
    port: env.PORT,

    mongoUri: env.MONGO_URI,

    jwt: {
      accessSecret: env.JWT_ACCESS_SECRET,
      refreshSecret: env.JWT_REFRESH_SECRET,
      accessTtl: env.ACCESS_TOKEN_TTL,
      refreshTtl: env.REFRESH_TOKEN_TTL,
    },

    adminEmails: env.ADMIN_EMAILS.split(",")
      .map((email) => email.trim().toLowerCase())
      .filter(Boolean),

    frontendUrl: env.FRONTEND_URL,
    cookieDomain: env.COOKIE_DOMAIN,

    google: {
      clientId: env.GOOGLE_CLIENT_ID,
      clientSecret: env.GOOGLE_CLIENT_SECRET,
      callbackUrl: env.GOOGLE_CALLBACK_URL,
    },

    email: {
      user: env.EMAIL_USER,
      appPassword: env.EMAIL_APP_PASSWORD,
    },

    otp: {
      expiryMinutes: env.OTP_EXPIRY_MINUTES,
    },

    resetToken: {
      secret: env.RESET_TOKEN_SECRET,
      ttl: env.RESET_TOKEN_TTL,
    },

    pinSetupToken: {
      secret: env.PIN_SETUP_TOKEN_SECRET,
      ttl: env.PIN_SETUP_TOKEN_TTL,
    },

    robotCall: {
      scheme: env.ROBOT_CALL_SCHEME,
      timeoutMs: env.ROBOT_CALL_TIMEOUT_MS,
    },
  };
}

export const config = loadConfig();
export type AppConfig = typeof config;
