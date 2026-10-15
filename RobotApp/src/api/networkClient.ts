import { ApiError } from "./apiError";

export interface WifiNetwork {
  ssid: string;
  signal: number;
  security: string;
}

export interface CurrentWifi {
  connected: boolean;
  ssid?: string;
}

export interface NetworkCommandResult {
  success: boolean;
  message: string;
}

/**
 * Plain fetch wrapper for RobotServer's /api/system/network/* endpoints.
 * Unauthenticated (same as /api/hmi-auth's pre-login routes) — wifi may
 * need setting up before anyone can even log in.
 */
async function request<T>(path: string, options: RequestInit = {}): Promise<T> {
  const response = await fetch(`/api/system/network${path}`, {
    method: "GET",
    headers: options.body ? { "Content-Type": "application/json" } : undefined,
    ...options,
  });

  const payload = await response.json().catch(() => undefined);

  // 502 is a well-formed { success: false, message } domain result (nmcli
  // rejected the connection, systemctl rejected the VPN toggle, etc.), not
  // a thrown error.
  if (!response.ok && response.status !== 502) {
    throw new ApiError(payload?.message ?? "Something went wrong", response.status);
  }

  return payload as T;
}

export const networkClient = {
  listWifiNetworks: () => request<{ networks: WifiNetwork[] }>("/wifi"),

  getCurrentWifi: () => request<CurrentWifi>("/wifi/current"),

  connectWifi: (ssid: string, password?: string) =>
    request<NetworkCommandResult>("/wifi/connect", { method: "POST", body: JSON.stringify({ ssid, password }) }),

  controlVpn: (action: "connect" | "disconnect") =>
    request<NetworkCommandResult>("/vpn", { method: "POST", body: JSON.stringify({ action }) }),
};
