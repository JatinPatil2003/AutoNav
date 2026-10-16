import type { ControlSessionClient } from "./controlSession.client";

/**
 * Process-wide pointer to whichever ControlSessionClient is currently live
 * (at most one — ControlPage only ever mounts one at a time). Exists so
 * the background-safety listeners (native/backgroundSafety.ts), which are
 * registered once at app startup and have no React tree of their own to
 * read from, can reach the live session without prop-drilling it through
 * the whole app.
 */
let active: ControlSessionClient | null = null;

export function setActiveControlSession(client: ControlSessionClient | null): void {
  active = client;
}

export function getActiveControlSession(): ControlSessionClient | null {
  return active;
}
