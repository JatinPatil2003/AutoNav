import { ApiError } from "./apiError";

export interface IdentifyResult {
  status: "has-access" | "no-account" | "no-access";
  pinSet: boolean;
  name?: string;
  avatarUrl?: string;
}

export interface SessionUser {
  userId: string;
  email: string;
  name: string;
  avatarUrl?: string;
}

export interface RosterUser {
  email: string;
  name: string;
  avatarUrl?: string;
  pinSet: boolean;
}

/**
 * Plain fetch wrapper for RobotServer's /api/hmi-auth/* endpoints. No
 * Redux/RTK Query here — RobotApp is a handful of screens, not a dashboard,
 * and it's always same-origin to RobotServer, so "credentials" is just the
 * browser's default cookie behavior (no CORS/SameSite handling needed, see
 * RobotServer/src/app.ts's static-mount comment).
 */

// Endpoints that are themselves part of the auth handshake — a 401 here
// means "not logged in yet", not "session expired", so retrying them after
// a refresh attempt would just loop.
const NO_REFRESH_PATHS = new Set([
  "/login",
  "/refresh",
  "/logout",
  "/identify",
  "/pin-setup/request-otp",
  "/pin-setup/verify-otp",
  "/pin-setup/confirm",
]);

// Single-flight: concurrent 401s (e.g. the WS auth check and a REST call
// landing together) share one refresh attempt instead of racing the robot
// refresh-token's rotation — mirrors PlatformApp's baseQueryWithReauth.
let refreshPromise: Promise<boolean> | null = null;

function attemptRefresh(): Promise<boolean> {
  if (!refreshPromise) {
    refreshPromise = fetch("/api/hmi-auth/refresh", { method: "POST" })
      .then((response) => response.ok)
      .catch(() => false)
      .finally(() => {
        refreshPromise = null;
      });
  }
  return refreshPromise;
}

async function request<T>(path: string, options: RequestInit = {}, isRetry = false): Promise<T> {
  const response = await fetch(`/api/hmi-auth${path}`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    ...options,
  });

  if (response.status === 401 && !isRetry && !NO_REFRESH_PATHS.has(path)) {
    const refreshed = await attemptRefresh();
    if (refreshed) return request<T>(path, options, true);
  }

  if (!response.ok) {
    const body = await response.json().catch(() => undefined);
    throw new ApiError(body?.message ?? "Something went wrong", response.status);
  }

  if (response.status === 204) return undefined as T;
  return response.json() as Promise<T>;
}

export const hmiClient = {
  listUsers: () => request<{ users: RosterUser[] }>("/users", { method: "GET" }),

  identify: (email: string) => request<IdentifyResult>("/identify", { body: JSON.stringify({ email }) }),

  login: (email: string, pin: string) =>
    request<{ email: string }>("/login", { body: JSON.stringify({ email, pin }) }),

  logout: () => request<void>("/logout"),

  session: () => request<{ user: SessionUser }>("/session", { method: "GET" }),

  requestPinSetupOtp: (email: string) =>
    request<{ message: string }>("/pin-setup/request-otp", { body: JSON.stringify({ email }) }),

  verifyPinSetupOtp: (email: string, otp: string) =>
    request<{ pinSetupToken: string }>("/pin-setup/verify-otp", {
      body: JSON.stringify({ email, otp }),
    }),

  confirmPinSetup: (pinSetupToken: string, pin: string) =>
    request<{ email: string }>("/pin-setup/confirm", {
      body: JSON.stringify({ pinSetupToken, pin }),
    }),
};
