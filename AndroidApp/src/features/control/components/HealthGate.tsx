import { AlertTriangle, Loader2, Wifi, WifiOff } from "lucide-react";
import { Button } from "../../../components/ui/Button";
import { Card } from "../../../components/ui/Card";
import { RobotHealth } from "../control.types";

interface HealthGateProps {
  health: RobotHealth | undefined;
  hasError: boolean;
  isChecking: boolean;
  onRetry: () => void;
}

/**
 * Shown before the joystick ever appears — the "robot health check" gate
 * the backend plan calls for, surfacing which specific check (link vs
 * ros) is failing rather than a generic "offline" message.
 */
export function HealthGate({ health, hasError, isChecking, onRetry }: HealthGateProps) {
  return (
    <div className="flex min-h-full flex-col items-center justify-center gap-4 p-6 text-center">
      <Card className="w-full max-w-sm">
        <span className="mx-auto mb-3 flex h-12 w-12 items-center justify-center rounded-xl bg-gradient-to-br from-brand-400 to-brand-600 shadow-md shadow-brand-900/20">
          {isChecking ? (
            <Loader2 className="h-6 w-6 animate-spin text-white" />
          ) : (
            <AlertTriangle className="h-6 w-6 text-white" />
          )}
        </span>
        <h2 className="text-lg font-semibold text-slate-900 dark:text-slate-100">
          {health && !health.checks.link.ok
            ? "Robot offline"
            : hasError
              ? "Couldn't reach the robot"
              : "Checking robot status…"}
        </h2>

        {/* The robot link being down makes the ROS check meaningless (it's
            never even queried — see RobotHealthService.getHealth) — so
            there's nothing useful to show beyond the one line above. The
            ROS row only matters once the link itself is up. */}
        {health && !health.checks.link.ok && (
          <p className="mt-2 flex items-center justify-center gap-2 text-sm text-red-600 dark:text-red-400">
            <WifiOff className="h-4 w-4 flex-shrink-0" />
            Can't reach this robot right now.
          </p>
        )}

        {health && health.checks.link.ok && (
          <ul className="mt-4 flex flex-col gap-2 text-left text-sm">
            <li className="flex items-center gap-2">
              <Wifi className="h-4 w-4 flex-shrink-0 text-brand-600 dark:text-brand-400" />
              <span className="text-slate-700 dark:text-slate-300">
                Robot link connected
                {health.checks.link.rttMs !== undefined ? ` (${health.checks.link.rttMs}ms)` : ""}
              </span>
            </li>
            <li className="flex items-center gap-2">
              <span
                className={`h-2 w-2 flex-shrink-0 rounded-full ${
                  health.checks.ros.ok ? "bg-brand-500" : "bg-red-500"
                }`}
              />
              <span
                className={
                  health.checks.ros.ok
                    ? "text-slate-700 dark:text-slate-300"
                    : "text-red-600 dark:text-red-400"
                }
              >
                ROS {health.checks.ros.ok ? "responding" : "not responding"}
              </span>
            </li>
          </ul>
        )}

        {!health && hasError && (
          <p className="mt-3 text-sm text-slate-500 dark:text-slate-400">
            Could not load a health report for this robot.
          </p>
        )}

        <Button className="mt-5 w-full" variant="secondary" onClick={onRetry} isLoading={isChecking}>
          Check again
        </Button>
      </Card>
    </div>
  );
}
