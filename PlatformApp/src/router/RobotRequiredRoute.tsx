import { PropsWithChildren } from "react";
import { Navigate } from "react-router-dom";
import { useAuth } from "../features/auth/useAuth";
import { ProtectedRoute } from "./ProtectedRoute";

/**
 * Gates the main dashboard shell: an authenticated non-admin user with no
 * robot assigned yet is redirected to the request-access screen instead.
 * Admins aren't expected to have a robot of their own, so they skip this
 * gate and go straight to the dashboard shell.
 */
export function RobotRequiredRoute({ children }: PropsWithChildren) {
  const { hasRobot, isAdmin } = useAuth();

  return (
    <ProtectedRoute>
      {!hasRobot && !isAdmin ? (
        <Navigate to="/request-access" replace />
      ) : (
        <>{children}</>
      )}
    </ProtectedRoute>
  );
}
