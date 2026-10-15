import { useCallback, useEffect, useState } from "react";
import { hmiClient, SessionUser } from "../../api/hmiClient";

type SessionStatus = "loading" | "authenticated" | "unauthenticated";

export function useSession() {
  const [status, setStatus] = useState<SessionStatus>("loading");
  const [user, setUser] = useState<SessionUser | null>(null);

  const refresh = useCallback(async () => {
    try {
      const { user: sessionUser } = await hmiClient.session();
      setUser(sessionUser);
      setStatus("authenticated");
    } catch {
      setUser(null);
      setStatus("unauthenticated");
    }
  }, []);

  const logout = useCallback(async () => {
    await hmiClient.logout().catch(() => undefined);
    setUser(null);
    setStatus("unauthenticated");
  }, []);

  // A reload must always ask for the PIN again, even if the session cookie
  // from before the reload is still valid — so this clears it outright on
  // mount instead of silently trusting it the way `refresh()` would. The
  // only path back to "authenticated" is an explicit PIN submit, which
  // calls `refresh()` itself after login (see App.tsx).
  useEffect(() => {
    logout();
  }, [logout]);

  return { status, user, refresh, logout };
}
