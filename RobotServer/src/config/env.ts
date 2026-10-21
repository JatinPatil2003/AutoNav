import "dotenv/config";
import { z } from "zod";

const envSchema = z.object({
  NODE_ENV: z.enum(["development", "test", "production"]).default("development"),
  PORT: z.coerce.number().int().positive().default(8000),

  MONGO_URI: z.string().min(1, "MONGO_URI is required"),

  CORS_ORIGINS: z.string().default(""),

  MAPS_DIR: z.string().default("./maps"),

  // Small persisted runtime state that must survive a process restart but
  // isn't a "map" — currently just the rotated ROBOT_LINK_SECRET override
  // (see config/linkSecretStore.ts). Mounted the same way MAPS_DIR is.
  RUNTIME_DIR: z.string().default("./runtime"),

  CAMERA_DEVICE: z.string().default("/dev/robotcam"),

  // --- Service-to-service (this robot <-> PlatformServer) ---
  // No secret: the VPN is the trust boundary (see root README). This robot
  // identifies itself to PlatformServer with its own device id, resolved
  // from the ESP32 bridge at boot (see ros/deviceIdentity.ts) — never from
  // config, except for local dev below.
  PLATFORM_BASE_URL: z.string().url(),
  ROSTER_SYNC_INTERVAL_MINUTES: z.coerce.number().int().positive().default(60),

  // --- Robot-link (this robot <-> PlatformServer's /ws/robot-link, see
  // clients/platformLink.client.ts) — the one call that DOES need a secret,
  // since it's what makes "robot online" a trusted, push-based fact rather
  // than a VPN-membership guess. Plaintext value issued once by
  // `POST /api/admin/robots` at provisioning time; only its sha256 hash is
  // ever stored on PlatformServer.
  ROBOT_LINK_SECRET: z.string().min(1, "ROBOT_LINK_SECRET is required"),

  // --- Local dev only: lets RobotServer boot without a real ESP32 attached ---
  DEV_DEVICE_ID_OVERRIDE: z.string().optional(),

  // --- Teleop safety limits (see modules/teleop/teleop.service.ts) ---
  // Defaults match autonav_navigation/config/navigation.yaml's DWB
  // max_velocity: [0.4, 0.0, 1.1] — the same safe envelope Nav2 itself is
  // configured to stay inside, so remote teleop can't exceed what the
  // robot is already tuned to do autonomously.
  TELEOP_MAX_LINEAR: z.coerce.number().positive().default(0.4),
  TELEOP_MAX_ANGULAR: z.coerce.number().positive().default(1.1),

  // --- RobotApp (local HMI PWA) human session ---
  ROBOT_SESSION_ACCESS_SECRET: z.string().min(16, "ROBOT_SESSION_ACCESS_SECRET must be at least 16 characters"),
  ROBOT_SESSION_REFRESH_SECRET: z.string().min(16, "ROBOT_SESSION_REFRESH_SECRET must be at least 16 characters"),
  ROBOT_SESSION_ACCESS_TTL: z.string().default("15m"),
  ROBOT_SESSION_REFRESH_TTL: z.string().default("24h"),
  PIN_LOGIN_MAX_ATTEMPTS: z.coerce.number().int().positive().default(5),
  PIN_LOGIN_LOCKOUT_MINUTES: z.coerce.number().int().positive().default(15),
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
    runtimeDir: env.RUNTIME_DIR,

    cameraDevice: env.CAMERA_DEVICE,

    platform: {
      baseUrl: env.PLATFORM_BASE_URL,
    },
    rosterSyncIntervalMinutes: env.ROSTER_SYNC_INTERVAL_MINUTES,

    robotLinkSecret: env.ROBOT_LINK_SECRET,

    devDeviceIdOverride: env.DEV_DEVICE_ID_OVERRIDE,

    teleop: {
      maxLinear: env.TELEOP_MAX_LINEAR,
      maxAngular: env.TELEOP_MAX_ANGULAR,
    },

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
  };
}

export const config = loadConfig();
export type AppConfig = typeof config;
