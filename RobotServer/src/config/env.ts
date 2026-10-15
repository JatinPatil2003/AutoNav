import "dotenv/config";
import { z } from "zod";

const envSchema = z.object({
  NODE_ENV: z.enum(["development", "test", "production"]).default("development"),
  PORT: z.coerce.number().int().positive().default(8000),

  MONGO_URI: z.string().min(1, "MONGO_URI is required"),

  CORS_ORIGINS: z.string().default(""),

  MAPS_DIR: z.string().default("./maps"),

  CAMERA_DEVICE: z.string().default("/dev/robotcam"),

  // --- Service-to-service (this robot <-> PlatformServer) ---
  // No secret: the VPN is the trust boundary (see root README). This robot
  // identifies itself to PlatformServer with its own device id, resolved
  // from the ESP32 bridge at boot (see ros/deviceIdentity.ts) — never from
  // config, except for local dev below.
  PLATFORM_BASE_URL: z.string().url(),
  ROSTER_SYNC_INTERVAL_MINUTES: z.coerce.number().int().positive().default(60),

  // --- Local dev only: lets RobotServer boot without a real ESP32 attached ---
  DEV_DEVICE_ID_OVERRIDE: z.string().optional(),

  // --- RobotApp (local HMI PWA) human session ---
  ROBOT_SESSION_ACCESS_SECRET: z.string().min(16, "ROBOT_SESSION_ACCESS_SECRET must be at least 16 characters"),
  ROBOT_SESSION_REFRESH_SECRET: z.string().min(16, "ROBOT_SESSION_REFRESH_SECRET must be at least 16 characters"),
  ROBOT_SESSION_ACCESS_TTL: z.string().default("15m"),
  ROBOT_SESSION_REFRESH_TTL: z.string().default("24h"),
  PIN_LOGIN_MAX_ATTEMPTS: z.coerce.number().int().positive().default(5),
  PIN_LOGIN_LOCKOUT_MINUTES: z.coerce.number().int().positive().default(15),

  // --- RobotApp static bundle ---
  ROBOTAPP_STATIC_DIR: z.string().default("./robotapp-dist"),
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

    corsOrigins: env.CORS_ORIGINS.split(",")
      .map((origin) => origin.trim())
      .filter(Boolean),

    mapsDir: env.MAPS_DIR,

    cameraDevice: env.CAMERA_DEVICE,

    platform: {
      baseUrl: env.PLATFORM_BASE_URL,
    },
    rosterSyncIntervalMinutes: env.ROSTER_SYNC_INTERVAL_MINUTES,

    devDeviceIdOverride: env.DEV_DEVICE_ID_OVERRIDE,

    robotSession: {
      accessSecret: env.ROBOT_SESSION_ACCESS_SECRET,
      refreshSecret: env.ROBOT_SESSION_REFRESH_SECRET,
      accessTtl: env.ROBOT_SESSION_ACCESS_TTL,
      refreshTtl: env.ROBOT_SESSION_REFRESH_TTL,
    },
    pinLogin: {
      maxAttempts: env.PIN_LOGIN_MAX_ATTEMPTS,
      lockoutMinutes: env.PIN_LOGIN_LOCKOUT_MINUTES,
    },

    robotAppStaticDir: env.ROBOTAPP_STATIC_DIR,
  };
}

export const config = loadConfig();
export type AppConfig = typeof config;
