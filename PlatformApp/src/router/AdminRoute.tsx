import { PropsWithChildren } from "react";
import { Navigate } from "react-router-dom";
import { FullPageSpinner } from "../components/ui/Spinner";
import { useAuth } from "../features/auth/useAuth";
import { ProtectedRoute } from "./ProtectedRoute";

export function AdminRoute({ children }: PropsWithChildren) {
  const { status, isAdmin } = useAuth();

  return (
    <ProtectedRoute>
      {status === "loading" ? (
        <FullPageSpinner />
      ) : !isAdmin ? (
        <Navigate to="/" replace />
      ) : (
        <>{children}</>
      )}
    </ProtectedRoute>
  );
}
