import { useState } from "react";
import { GoogleSignIn } from "@capawesome/capacitor-google-sign-in";
import { Button } from "../../../components/ui/Button";
import { ensureGoogleSignInInitialized } from "../../../native/googleSignIn";
import { useGetMeQuery, useGoogleTokenMutation } from "../authApi";

interface GoogleSignInButtonProps {
  onSuccess: () => void;
}

function isCanceledSignIn(err: unknown): boolean {
  return Boolean(err) && typeof err === "object" && "code" in (err as object) &&
    (err as { code?: string }).code === "SIGN_IN_CANCELED";
}

/**
 * Native Credential Manager sign-in (no WebView redirect) — the Android
 * counterpart to PlatformApp's browser-redirect GoogleAuthButton. Exchanges
 * the device-native ID token for an AutoNav session via
 * POST /auth/google/token, verified server-side in auth.controller.ts.
 */
export function GoogleSignInButton({ onSuccess }: GoogleSignInButtonProps) {
  const [googleToken, { isLoading }] = useGoogleTokenMutation();
  const { refetch: refetchMe } = useGetMeQuery();
  const [error, setError] = useState<string | null>(null);
  const [isSigningIn, setIsSigningIn] = useState(false);

  async function handlePress() {
    setError(null);
    setIsSigningIn(true);
    try {
      await ensureGoogleSignInInitialized();
      const result = await GoogleSignIn.signIn();
      await googleToken({ idToken: result.idToken }).unwrap();
      // Same reasoning as LoginForm/SignupForm: a pre-login cached 401
      // getMe() result won't auto-refetch from tag invalidation alone.
      await refetchMe().unwrap();
      onSuccess();
    } catch (err) {
      if (!isCanceledSignIn(err)) {
        setError(err instanceof Error ? err.message : "Google sign-in failed");
      }
    } finally {
      setIsSigningIn(false);
    }
  }

  return (
    <div className="flex flex-col gap-2">
      <Button
        type="button"
        variant="secondary"
        isLoading={isLoading || isSigningIn}
        className="w-full gap-2"
        onClick={handlePress}
      >
        <GoogleGlyph />
        Continue with Google
      </Button>
      {error && <p className="text-sm text-red-600 dark:text-red-400">{error}</p>}
    </div>
  );
}

function GoogleGlyph() {
  return (
    <svg aria-hidden="true" viewBox="0 0 24 24" className="h-4 w-4">
      <path
        fill="#4285F4"
        d="M23.52 12.27c0-.79-.07-1.54-.2-2.27H12v4.3h6.47c-.28 1.5-1.13 2.77-2.4 3.62v3.01h3.49c2.04-1.88 3.23-4.65 3.23-8.66z"
      />
      <path
        fill="#34A853"
        d="M12 24c3.24 0 5.95-1.07 7.93-2.9l-3.49-3.01c-1.03.7-2.4 1.1-4.44 1.1-3.42 0-6.32-2.3-7.35-5.4H1.07v3.1C3.03 21.3 7.2 24 12 24z"
      />
      <path
        fill="#FBBC05"
        d="M4.65 14.79c-.26-.78-.4-1.62-.4-2.49s.14-1.71.4-2.49V6.72H1.07A11.98 11.98 0 0 0 0 12.3c0 1.94.46 3.78 1.07 5.4z"
      />
      <path
        fill="#EA4335"
        d="M12 4.75c1.76 0 3.34.6 4.58 1.78l3.1-3.1C17.94 1.7 15.23.6 12 .6 7.2.6 3.03 3.3 1.07 6.72l3.58 2.78c1.03-3.1 3.93-5.4 7.35-5.4z"
      />
    </svg>
  );
}
