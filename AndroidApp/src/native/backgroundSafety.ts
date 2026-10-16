import { App as CapacitorApp } from "@capacitor/app";
import { Capacitor } from "@capacitor/core";
import { getActiveControlSession } from "../features/control/activeControlSession";

/**
 * Defense-in-depth client-side kill switch: the instant the app is
 * backgrounded (or the WebView is torn down), synchronously fire an
 * e-stop and close the control session, rather than waiting for the
 * robot-side per-command TTL watchdog (Part 3 of the backend plan, ~500ms)
 * to expire on its own. That server/robot-side watchdog remains the real
 * safety net — this is strictly a faster, best-effort reaction on the
 * happy path, not something safety depends on.
 */
function stopActiveSession(): void {
  const session = getActiveControlSession();
  if (!session) return;
  session.sendEstopImmediate();
  session.close();
}

export function registerBackgroundSafety(): void {
  if (Capacitor.isNativePlatform()) {
    // Fires on Android's Activity.onPause — before the WebView is throttled.
    void CapacitorApp.addListener("pause", stopActiveSession);
  }

  // Web fallback (also exercised when running via `npm run dev` in a
  // browser tab, or inside the WebView's own visibilitychange semantics).
  document.addEventListener("visibilitychange", () => {
    if (document.visibilityState === "hidden") stopActiveSession();
  });

  window.addEventListener("beforeunload", stopActiveSession);
}
