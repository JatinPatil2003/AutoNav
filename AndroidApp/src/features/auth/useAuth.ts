import { useGetMeQuery } from "./authApi";

export type AuthStatus = "loading" | "authenticated" | "unauthenticated";

export interface UseAuthResult {
  status: AuthStatus;
  user: ReturnType<typeof useGetMeQuery>["data"];
  hasRobot: boolean;
}

/**
 * Single source of truth for "who is logged in" — backed directly by RTK
 * Query's cache for GET /users/me rather than a parallel Redux slice, so
 * there is nothing to manually keep in sync after login/logout/refresh.
 */
export function useAuth(): UseAuthResult {
  const { data: user, isLoading, isUninitialized, isError } = useGetMeQuery();

  const status: AuthStatus =
    isLoading || isUninitialized ? "loading" : isError || !user ? "unauthenticated" : "authenticated";

  return {
    status,
    user,
    hasRobot: (user?.robots.length ?? 0) > 0,
  };
}
