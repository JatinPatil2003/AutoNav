import { PropsWithChildren, useCallback, useEffect, useState } from "react";
import { useAuth } from "../../features/auth/useAuth";
import { RobotSummary } from "../../features/auth/auth.types";
import { readPreference, writePreference } from "../../native/preferences";
import { RobotSelectionContext } from "./robotSelection.context";

const STORAGE_KEY = "autonav-selected-robot";

/**
 * Which of the current user's (possibly many) robots the app is currently
 * "operating". Persisted per-user via Capacitor Preferences (real device
 * storage, not raw localStorage — a Capacitor WebView's localStorage isn't
 * guaranteed durable the way native app storage is), keyed
 * "autonav-selected-robot:<userId>" to match PlatformApp's key format.
 *
 * Unlike PlatformApp's dashboard equivalent, this never silently defaults
 * to robots[0]: if nothing valid is stored and the user has more than one
 * robot, `selectedRobotId` stays null so the router can send them to an
 * explicit picker (RobotSelectPage) instead of guessing which robot to
 * arm for teleop.
 */
export function RobotSelectionProvider({ children }: PropsWithChildren) {
  const { user } = useAuth();
  const robots: RobotSummary[] = user?.robots ?? [];
  const robotIdsKey = robots.map((r) => r.id).join(",");

  const [selectedRobotId, setSelectedRobotIdState] = useState<string | null>(null);
  const [isReady, setIsReady] = useState(false);

  useEffect(() => {
    let cancelled = false;

    async function load() {
      setIsReady(false);

      if (!user || robots.length === 0) {
        if (!cancelled) {
          setSelectedRobotIdState(null);
          setIsReady(true);
        }
        return;
      }

      const stored = await readPreference(`${STORAGE_KEY}:${user.id}`);
      if (cancelled) return;

      const isStillValid = stored !== null && robots.some((r) => r.id === stored);

      if (isStillValid) {
        setSelectedRobotIdState(stored);
      } else if (robots.length === 1) {
        // Only one possible robot — nothing to ask the user, select it
        // outright and persist that as an explicit choice.
        setSelectedRobotIdState(robots[0].id);
        void writePreference(`${STORAGE_KEY}:${user.id}`, robots[0].id);
      } else {
        setSelectedRobotIdState(null);
      }
      setIsReady(true);
    }

    void load();
    return () => {
      cancelled = true;
    };
    // Only re-derive when the user or their robot list actually changes.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [user?.id, robotIdsKey]);

  const setSelectedRobotId = useCallback(
    (robotId: string) => {
      setSelectedRobotIdState(robotId);
      if (user) void writePreference(`${STORAGE_KEY}:${user.id}`, robotId);
    },
    [user],
  );

  return (
    <RobotSelectionContext.Provider value={{ selectedRobotId, setSelectedRobotId, isReady }}>
      {children}
    </RobotSelectionContext.Provider>
  );
}
