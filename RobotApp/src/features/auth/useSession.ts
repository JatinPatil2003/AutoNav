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

  useEffect(() => {
    refresh();
  }, [refresh]);

  const logout = useCallback(async () => {
    await hmiClient.logout().catch(() => undefined);
    setUser(null);
    setStatus("unauthenticated");
  }, []);

  return { status, user, refresh, logout };
}
