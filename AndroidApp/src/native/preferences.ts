import { Preferences } from "@capacitor/preferences";

/**
 * Small, non-sensitive, best-effort key/value persistence for things like
 * "which robot did the user pick last" or "what velocity multiplier did
 * they set" — real device storage via Capacitor Preferences, not raw
 * localStorage (a Capacitor WebView's localStorage isn't guaranteed to
 * survive the way native app storage is). Never used for credentials —
 * see app/tokenStore.ts for that.
 */

export async function readPreference(key: string): Promise<string | null> {
  try {
    const { value } = await Preferences.get({ key });
    return value;
  } catch {
    return null;
  }
}

export async function writePreference(key: string, value: string): Promise<void> {
  try {
    await Preferences.set({ key, value });
  } catch {
    // Best-effort — a failed write just means the choice isn't remembered
    // next launch, not worth surfacing to the user.
  }
}

export async function removePreference(key: string): Promise<void> {
  try {
    await Preferences.remove({ key });
  } catch {
    // ignore
  }
}
