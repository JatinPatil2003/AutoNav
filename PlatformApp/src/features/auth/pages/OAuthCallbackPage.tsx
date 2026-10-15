import { useEffect } from "react";
import { useNavigate } from "react-router-dom";
import { FullPageSpinner } from "../../../components/ui/Spinner";
import { useGetMeQuery } from "../authApi";

/**
 * Landing target for the Google OAuth redirect (PlatformServer sets auth
 * cookies then 302s here). The access/refresh cookies already exist by the
 * time this page loads — this just needs to re-fetch /me (any pre-login
 * "unauthenticated" result may still be cached) before routing onward.
 */
export function OAuthCallbackPage() {
  const navigate = useNavigate();
  const { refetch } = useGetMeQuery();

  useEffect(() => {
    refetch()
      .unwrap()
      .then(() => navigate("/", { replace: true }))
      .catch(() => navigate("/login", { replace: true }));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  return <FullPageSpinner />;
}
