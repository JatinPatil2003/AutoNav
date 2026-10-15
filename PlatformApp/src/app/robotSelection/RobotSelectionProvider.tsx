import { PropsWithChildren, useCallback, useEffect, useState } from "react";
import { useAuth } from "../../features/auth/useAuth";
import { RobotSummary } from "../../features/auth/auth.types";
import { RobotSelectionContext } from "./robotSelection.context";

const STORAGE_KEY = "autonav-selected-robot";

function readStored(userId: string): string | null {
  try {
    return localStorage.getItem(`${STORAGE_KEY}:${userId}`);
  } catch {
    return null;
  }
}

function writeStored(userId: string, robotId: string): void {
  try {
    localStorage.setItem(`${STORAGE_KEY}:${userId}`, robotId);
  } catch {
    // ignore write failures (private browsing, storage disabled, etc.)
  }
}

/**
 * Which of the current user's (possibly many) robots the dashboard is
 * currently "operating". Remembered per-user in localStorage so it
 * survives a reload, but always falls back to the first available robot
 * if the stored choice is no longer in the user's robot list (e.g. access
 * to that robot was revoked).
 */
export function RobotSelectionProvider({ children }: PropsWithChildren) {
  const { user } = useAuth();
  const robots: RobotSummary[] = user?.robots ?? [];
  const [selectedRobotId, setSelectedRobotIdState] = useState<string | null>(null);

  useEffect(() => {
    if (!user || robots.length === 0) {
      setSelectedRobotIdState(null);
      return;
    }

    const stored = readStored(user.id);
    const isStillValid = stored && robots.some((r: RobotSummary) => r.id === stored);
    setSelectedRobotIdState(isStillValid ? stored : robots[0].id);
    // Only re-derive when the user or their robot list actually changes.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [user?.id, robots.map((r: RobotSummary) => r.id).join(",")]);

  const setSelectedRobotId = useCallback(
    (robotId: string) => {
      setSelectedRobotIdState(robotId);
      if (user) writeStored(user.id, robotId);
    },
    [user],
  );

  return (
    <RobotSelectionContext.Provider value={{ selectedRobotId, setSelectedRobotId }}>
      {children}
    </RobotSelectionContext.Provider>
  );
}
