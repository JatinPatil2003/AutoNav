import { App as CapacitorApp } from "@capacitor/app";
import { Capacitor } from "@capacitor/core";
import { Radio, RadioTower, WifiOff } from "lucide-react";
import { useEffect, useState } from "react";
import { useSelectedRobot } from "../../../app/robotSelection/useSelectedRobot";
import { Card } from "../../../components/ui/Card";
import { FullPageSpinner } from "../../../components/ui/Spinner";
import { useVelocitySettings } from "../../settings/useVelocitySettings";
import { EstopButton } from "../components/EstopButton";
import { HealthGate } from "../components/HealthGate";
import { JoystickPad } from "../components/JoystickPad";
import { useGetRobotHealthQuery } from "../healthApi";
import { useControlSession } from "../useControlSession";

const HEALTH_POLL_MS = 4000;

function statusLabel(status: string, estopEngaged: boolean): string {
  if (estopEngaged) return "Stopped";
  switch (status) {
    case "open":
      return "Connected";
    case "connecting":
      return "Connecting…";
    case "reconnecting":
      return "Reconnecting…";
    default:
      return "Disconnected";
  }
}

function statusTone(status: string, estopEngaged: boolean): "live" | "busy" | "down" {
  if (estopEngaged) return "busy";
  if (status === "open") return "live";
  if (status === "connecting" || status === "reconnecting") return "busy";
  return "down";
}

/**
 * The core teleop screen: health-gates on GET /robots/:id/health until the
 * robot reports healthy, then opens the /ws/control session and renders
 * the joystick + e-stop. A velocity multiplier from Settings is applied to
 * outgoing stick values before they're sent.
 */
export function ControlPage() {
  const { selectedRobot } = useSelectedRobot();
  const robotId = selectedRobot?.id ?? null;
  const { linearMultiplier, angularMultiplier } = useVelocitySettings();
  const [estopEngaged, setEstopEngaged] = useState(false);
  const [retrySignal, setRetrySignal] = useState(0);

  const {
    data: health,
    error: healthError,
    isFetching: isCheckingHealth,
    refetch: refetchHealth,
  } = useGetRobotHealthQuery(robotId ?? "", {
    skip: !robotId,
    // Keep polling even once connected, so a robot going offline
    // mid-session is reflected here too.
    pollingInterval: HEALTH_POLL_MS,
  });

  const sessionActive = Boolean(robotId) && health?.healthy === true;
  const { status, lastServerFrame, sendJoystick, sendEstop } = useControlSession(
    robotId,
    sessionActive,
    retrySignal,
  );

  useEffect(() => {
    if (lastServerFrame?.type === "estop_ack") {
      setEstopEngaged(lastServerFrame.engaged);
    }
  }, [lastServerFrame]);

  // The robot reports its own real velocity envelope via the health check
  // (RobotServer's TeleopService.getLimits(), relayed through
  // PlatformServer) — scaling the joystick's -1..1 range against that
  // (rather than against an assumed constant) means full stick deflection
  // always reaches the robot's actual max, with nothing wasted to a
  // mismatched clamp partway through the stick's travel. The `?? 1`
  // fallback only matters against an older RobotServer that hasn't picked
  // up this field yet.
  const maxLinear = health?.checks.ros.maxLinear ?? 1;
  const maxAngular = health?.checks.ros.maxAngular ?? 1;

  function handleMove(linear: number, angular: number): void {
    sendJoystick(linear * linearMultiplier * maxLinear, angular * angularMultiplier * maxAngular);
  }

  function handleStop(): void {
    sendJoystick(0, 0);
  }

  function handleEstopEngage(): void {
    setEstopEngaged(true);
    sendEstop(true);
  }

  function handleEstopRelease(): void {
    setEstopEngaged(false);
    sendEstop(false);
  }

  function handleRetryConnection(): void {
    setEstopEngaged(false);
    setRetrySignal((s) => s + 1);
  }

  // The app's background-safety kill switch (src/native/backgroundSafety.ts)
  // e-stops and terminally closes the session on every pause/hide, so
  // coming back to the foreground always needs a fresh connection —
  // otherwise the user is stuck on "Disconnected" until they navigate away
  // and back (the manual escape hatch `handleRetryConnection` provides).
  useEffect(() => {
    function reconnect(): void {
      setEstopEngaged(false);
      setRetrySignal((s) => s + 1);
      refetchHealth();
    }

    let removeResumeListener: (() => void) | undefined;
    if (Capacitor.isNativePlatform()) {
      const listenerPromise = CapacitorApp.addListener("resume", reconnect);
      void listenerPromise.then((handle) => {
        removeResumeListener = () => void handle.remove();
      });
    }

    function handleVisibility(): void {
      if (document.visibilityState === "visible") reconnect();
    }
    document.addEventListener("visibilitychange", handleVisibility);

    return () => {
      removeResumeListener?.();
      document.removeEventListener("visibilitychange", handleVisibility);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  if (!robotId) {
    // RobotRequiredRoute/RobotSelectPage guarantee a robot is selected
    // before this route can render.
    return <FullPageSpinner />;
  }

  if (!health?.healthy) {
    return (
      <HealthGate
        health={health}
        hasError={Boolean(healthError)}
        isChecking={isCheckingHealth}
        onRetry={refetchHealth}
      />
    );
  }

  const terminalSessionState =
    lastServerFrame?.type === "session" && lastServerFrame.state !== "ready" ? lastServerFrame.state : null;
  const tone = statusTone(status, estopEngaged);
  const toneStyles = {
    live: {
      badge: "from-brand-400 to-brand-600 shadow-brand-900/25",
      dot: "bg-brand-400",
      text: "text-brand-600 dark:text-brand-400",
      icon: Radio,
    },
    busy: {
      badge: "from-amber-400 to-amber-600 shadow-amber-900/25",
      dot: "bg-amber-400",
      text: "text-amber-600 dark:text-amber-400",
      icon: RadioTower,
    },
    down: {
      badge: "from-slate-400 to-slate-600 shadow-slate-900/25",
      dot: "bg-slate-400",
      text: "text-slate-500 dark:text-slate-400",
      icon: WifiOff,
    },
  }[tone];
  const StatusIcon = toneStyles.icon;

  return (
    <div className="flex h-full flex-col p-4 pb-safe-bottom">
      <Card className="w-full">
        <div className="flex items-center gap-3">
          <span
            className={`relative flex h-11 w-11 flex-shrink-0 items-center justify-center rounded-xl bg-gradient-to-br shadow-md ${toneStyles.badge}`}
          >
            <StatusIcon className="h-5 w-5 text-white" />
            {tone === "live" && (
              <span className={`absolute -right-1 -top-1 h-3 w-3 rounded-full ${toneStyles.dot} animate-pulse ring-2 ring-white dark:ring-slate-900`} />
            )}
          </span>
          <div className="min-w-0 flex-1">
            <p className="truncate text-sm font-semibold text-slate-900 dark:text-slate-100">
              {selectedRobot?.displayName ?? selectedRobot?.name}
            </p>
            <p className={`text-xs font-medium uppercase tracking-wide ${toneStyles.text}`}>
              {statusLabel(status, estopEngaged)}
            </p>
          </div>
        </div>
      </Card>

      {terminalSessionState ? (
        <div className="flex flex-1 items-center justify-center">
          <Card className="flex w-full max-w-sm flex-col items-center gap-3 py-8 text-center">
            <WifiOff className="h-8 w-8 text-slate-400 dark:text-slate-500" />
            <p className="text-sm text-slate-700 dark:text-slate-300">
              {sessionTerminalMessage(terminalSessionState)}
            </p>
            <button
              type="button"
              onClick={handleRetryConnection}
              className="mt-1 rounded-lg bg-brand-600 px-4 py-2 text-sm font-medium text-white transition-colors hover:bg-brand-700"
            >
              Try reconnecting
            </button>
          </Card>
        </div>
      ) : (
        // Both controls sit low and to the sides — within thumb reach while
        // gripping the phone with one hand — rather than centered, which is
        // what made the old layout awkward to drive one-handed.
        <div className="flex flex-1 items-end justify-between gap-4 pb-4">
          <EstopButton engaged={estopEngaged} onEngage={handleEstopEngage} onRelease={handleEstopRelease} />
          <JoystickPad
            disabled={status !== "open" || estopEngaged}
            onMove={handleMove}
            onStop={handleStop}
          />
        </div>
      )}
    </div>
  );
}

function sessionTerminalMessage(state: "robot_offline" | "preempted" | "revoked"): string {
  switch (state) {
    case "preempted":
      return "Another session took over control of this robot.";
    case "revoked":
      return "Your access to this robot was revoked.";
    case "robot_offline":
      return "The robot's connection to the server dropped.";
    default:
      return "The control session ended.";
  }
}
