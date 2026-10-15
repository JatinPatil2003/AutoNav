import { PropsWithChildren } from "react";
import { Navigate } from "react-router-dom";
import { FullPageSpinner } from "../components/ui/Spinner";
import { useAuth } from "../features/auth/useAuth";

export function ProtectedRoute({ children }: PropsWithChildren) {
  const { status } = useAuth();

  if (status === "loading") return <FullPageSpinner />;
  if (status === "unauthenticated") return <Navigate to="/login" replace />;

  return <>{children}</>;
}
