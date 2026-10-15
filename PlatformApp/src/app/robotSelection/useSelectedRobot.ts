import { useContext } from "react";
import { useAuth } from "../../features/auth/useAuth";
import { RobotSummary } from "../../features/auth/auth.types";
import { RobotSelectionContext } from "./robotSelection.context";

/** The currently selected robot (full object) plus the raw context for switching it. */
export function useSelectedRobot() {
  const ctx = useContext(RobotSelectionContext);
  if (!ctx) {
    throw new Error("useSelectedRobot must be used within a RobotSelectionProvider");
  }

  const { user } = useAuth();
  const robots: RobotSummary[] = user?.robots ?? [];
  const selectedRobot = robots.find((r: RobotSummary) => r.id === ctx.selectedRobotId) ?? null;

  return { ...ctx, robots, selectedRobot };
}
