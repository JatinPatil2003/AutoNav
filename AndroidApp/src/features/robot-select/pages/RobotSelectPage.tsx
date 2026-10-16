import { useNavigate } from "react-router-dom";
import { Bot } from "lucide-react";
import { Button } from "../../../components/ui/Button";
import { Card } from "../../../components/ui/Card";
import { useSelectedRobot } from "../../../app/robotSelection/useSelectedRobot";

/**
 * Forced on first launch whenever the signed-in user has more than one
 * assigned robot and no persisted selection exists yet (see
 * RobotSelectionProvider — unlike PlatformApp's dashboard, this app never
 * silently defaults to robots[0] for teleop). Also reachable later from
 * Settings to switch which robot is being driven.
 */
export function RobotSelectPage() {
  const { robots, selectedRobotId, setSelectedRobotId } = useSelectedRobot();
  const navigate = useNavigate();

  function handleSelect(robotId: string) {
    setSelectedRobotId(robotId);
    navigate("/control", { replace: true });
  }

  return (
    <div className="app-canvas flex min-h-screen flex-col items-center justify-center gap-4 p-6">
      <h1 className="text-lg font-semibold text-slate-900 dark:text-slate-100">Choose a robot</h1>
      <p className="text-center text-sm text-slate-500 dark:text-slate-400">
        You have access to more than one robot. Pick which one to operate.
      </p>

      <div className="flex w-full max-w-sm flex-col gap-3">
        {robots.map((robot) => (
          <Card
            key={robot.id}
            className={`flex items-center gap-3 ${
              robot.id === selectedRobotId ? "ring-2 ring-brand-500" : ""
            }`}
          >
            <span className="flex h-10 w-10 flex-shrink-0 items-center justify-center rounded-lg bg-gradient-to-br from-brand-400 to-brand-600">
              <Bot className="h-5 w-5 text-white" />
            </span>
            <div className="flex-1 text-left">
              <p className="font-medium text-slate-900 dark:text-slate-100">
                {robot.displayName ?? robot.name}
              </p>
              <p className="text-xs text-slate-400 dark:text-slate-500">{robot.vpnHostname}</p>
            </div>
            <Button onClick={() => handleSelect(robot.id)}>Select</Button>
          </Card>
        ))}
      </div>
    </div>
  );
}
