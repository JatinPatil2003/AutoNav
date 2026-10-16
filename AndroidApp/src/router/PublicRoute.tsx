import { PropsWithChildren } from "react";
import { Navigate } from "react-router-dom";
import { FullPageSpinner } from "../components/ui/Spinner";
import { useAuth } from "../features/auth/useAuth";

export function PublicRoute({ children }: PropsWithChildren) {
  const { status } = useAuth();

  if (status === "loading") return <FullPageSpinner />;
  if (status === "authenticated") return <Navigate to="/" replace />;

  return <>{children}</>;
}
