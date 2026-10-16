import { SecureStorage, StorageError } from "@aparajita/capacitor-secure-storage";

export interface TokenPair {
  accessToken: string;
  refreshToken: string;
}

const REFRESH_TOKEN_KEY = "autonav-refresh-token";

/**
 * The access token lives in memory only (re-derived via /auth/refresh on
 * cold start) and is never written to disk. The refresh token is the only
 * credential that survives a restart, and it lives in Android
 * Keystore-backed secure storage — never localStorage/Preferences, which
 * are not encrypted at rest inside a WebView.
 */
let accessToken: string | null = null;

async function getRefreshToken(): Promise<string | null> {
  try {
    const value = await SecureStorage.get(REFRESH_TOKEN_KEY);
    return typeof value === "string" ? value : null;
  } catch (err) {
    if (err instanceof StorageError) return null;
    throw err;
  }
}

async function setRefreshToken(token: string | null): Promise<void> {
  try {
    if (token) {
      await SecureStorage.set(REFRESH_TOKEN_KEY, token);
    } else {
      await SecureStorage.remove(REFRESH_TOKEN_KEY);
    }
  } catch {
    // Best-effort — if secure storage is unavailable the user is simply
    // asked to log in again on next cold start, an acceptable degradation
    // rather than a crash.
  }
}

export const tokenStore = {
  getAccessToken(): string | null {
    return accessToken;
  },

  setAccessToken(token: string | null): void {
    accessToken = token;
  },

  getRefreshToken,

  setRefreshToken,

  async setTokenPair(tokens: TokenPair): Promise<void> {
    accessToken = tokens.accessToken;
    await setRefreshToken(tokens.refreshToken);
  },

  async clear(): Promise<void> {
    accessToken = null;
    await setRefreshToken(null);
  },
};
