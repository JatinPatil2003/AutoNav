import { GoogleSignIn } from "@capawesome/capacitor-google-sign-in";

let initPromise: Promise<void> | null = null;

/**
 * `GoogleSignIn.initialize()` must run exactly once before `signIn()` is
 * ever called. Lazily memoized rather than fired eagerly at app startup so
 * a missing/blank VITE_GOOGLE_WEB_CLIENT_ID doesn't fail silently before
 * anyone has even tapped "Continue with Google" — the error surfaces at
 * the point of use instead.
 *
 * `clientId` must be the **web** OAuth client ID from Google Cloud
 * Console, not an Android client ID, on every platform — see
 * PlatformServer's verifyGoogleIdToken, whose `audience` check expects it.
 */
export function ensureGoogleSignInInitialized(): Promise<void> {
  initPromise ??= GoogleSignIn.initialize({
    clientId: import.meta.env.VITE_GOOGLE_WEB_CLIENT_ID,
  });
  return initPromise;
}
