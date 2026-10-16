import { PropsWithChildren } from "react";
import { Navigate } from "react-router-dom";
import { FullPageSpinner } from "../components/ui/Spinner";
import { useSelectedRobot } from "../app/robotSelection/useSelectedRobot";
import { useAuth } from "../features/auth/useAuth";
import { ProtectedRoute } from "./ProtectedRoute";

/**
 * Gates the main app shell:
 *  - an authenticated user with no robot assigned is sent to the
 *    request-access screen (same as PlatformApp's dashboard gate).
 *  - once robots exist, waits for RobotSelectionProvider to finish reading
 *    the persisted selection from device storage.
 *  - if more than one robot is assigned and nothing valid is stored yet,
 *    sends the user to an explicit picker (RobotSelectPage) instead of
 *    silently defaulting to the first robot — unlike PlatformApp's
 *    dashboard, this app is about to arm a specific physical robot for
 *    motion, so an implicit default would be the wrong default to pick.
 */
export function RobotRequiredRoute({ children }: PropsWithChildren) {
  const { hasRobot } = useAuth();
  const { robots, selectedRobotId, isReady } = useSelectedRobot();

  return (
    <ProtectedRoute>
      {!hasRobot ? (
        <Navigate to="/request-access" replace />
      ) : !isReady ? (
        <FullPageSpinner />
      ) : robots.length > 1 && !selectedRobotId ? (
        <Navigate to="/robot-select" replace />
      ) : (
        <>{children}</>
      )}
    </ProtectedRoute>
  );
}
